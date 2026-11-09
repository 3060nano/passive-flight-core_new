#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/PerturbedLongitudinalDynamics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {

/// Число обнаруженных нарушений тестовых условий, безразмерный счётчик.
int failureCount = 0;

/// Преобразует типизированный индекс компоненты в безразмерный индекс массива.
constexpr std::size_t indexOf(
    passive_flight::LongitudinalStateIndex index
) noexcept {
    return static_cast<std::size_t>(index);
}

/// Регистрирует нарушение логического условия без немедленного завершения.
void check(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        ++failureCount;
    }
}

/// Проверяет абсолютную погрешность величин в одинаковых единицах.
void checkNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr
            << "FAILED: " << message
            << "; expected " << expected
            << ", actual " << actual
            << ", tolerance " << tolerance
            << '\n';

        ++failureCount;
    }
}

/// Требует возникновения исключения ExceptionType при вызове function.
template <typename ExceptionType, typename Function>
void checkThrows(
    Function function,
    const std::string& message
) {
    try {
        function();

        std::cerr
            << "FAILED: " << message
            << "; exception was not thrown\n";

        ++failureCount;
    } catch (const ExceptionType&) {
        // Ожидаемое исключение.
    } catch (...) {
        std::cerr
            << "FAILED: " << message
            << "; unexpected exception type\n";

        ++failureCount;
    }
}

/// Создаёт линеаризованную динамику контрольной ФАБ-1500Т.
passive_flight::PerturbedLongitudinalDynamics
makeDynamics() {
    const auto passport =
        passive_flight::makeFab1500TPostnikovPassport();

    return passive_flight::PerturbedLongitudinalDynamics(
        passport.object
    );
}

/// Формирует номинальное состояние для проверки матрицы Якоби.
/// @return Состояние в порядке `t, V, Theta, omega_z, theta, x, H` и СИ.
passive_flight::State makeNominalState() {
    return {
        20.0,
        250.0,
        -0.20,
        -0.01,
        -0.18,
        4500.0,
        7000.0
    };
}

/// Проверяет нулевую производную при нулевом векторе возмущений.
void testZeroPerturbationHasZeroDerivative() {
    const auto dynamics = makeDynamics();

    const auto evaluation =
        dynamics.evaluate(
            makeNominalState(),
            {}
        );

    checkNear(
        evaluation.derivative.speedMps2,
        0.0,
        0.0,
        "Zero Delta V derivative"
    );

    checkNear(
        evaluation.derivative.flightPathAngleRadps,
        0.0,
        0.0,
        "Zero Delta Theta derivative"
    );

    checkNear(
        evaluation.derivative.pitchRateRadps2,
        0.0,
        0.0,
        "Zero Delta omega-z derivative"
    );

    checkNear(
        evaluation.derivative.pitchAngleRadps,
        0.0,
        0.0,
        "Zero Delta pitch derivative"
    );

    checkNear(
        evaluation.derivative.downrangeMps,
        0.0,
        0.0,
        "Zero Delta x derivative"
    );

    checkNear(
        evaluation.derivative.altitudeMps,
        0.0,
        0.0,
        "Zero Delta H derivative"
    );
}

/// Проверяет аналитически известные кинематические элементы матрицы Якоби.
void testKinematicMatrixTerms() {
    const auto dynamics = makeDynamics();
    const auto state = makeNominalState();
    const auto matrix = dynamics.linearize(state);

    const std::size_t speed = indexOf(
        passive_flight::LongitudinalStateIndex::Speed
    );
    const std::size_t flightPathAngle = indexOf(
        passive_flight::LongitudinalStateIndex::FlightPathAngle
    );
    const std::size_t pitchRate = indexOf(
        passive_flight::LongitudinalStateIndex::PitchRate
    );
    const std::size_t pitchAngle = indexOf(
        passive_flight::LongitudinalStateIndex::PitchAngle
    );
    const std::size_t downrange = indexOf(
        passive_flight::LongitudinalStateIndex::Downrange
    );
    const std::size_t altitude = indexOf(
        passive_flight::LongitudinalStateIndex::Altitude
    );

    checkNear(
        matrix[pitchAngle][pitchRate],
        1.0,
        1.0e-9,
        "d(Delta theta dot)/d(Delta omega-z)"
    );

    checkNear(
        matrix[downrange][speed],
        std::cos(state.flightPathAngleRad),
        1.0e-9,
        "d(Delta x dot)/d(Delta V)"
    );

    checkNear(
        matrix[downrange][flightPathAngle],
        -state.speedMps *
            std::sin(state.flightPathAngleRad),
        1.0e-5,
        "d(Delta x dot)/d(Delta Theta)"
    );

    checkNear(
        matrix[altitude][speed],
        std::sin(state.flightPathAngleRad),
        1.0e-9,
        "d(Delta H dot)/d(Delta V)"
    );

    checkNear(
        matrix[altitude][flightPathAngle],
        state.speedMps *
            std::cos(state.flightPathAngleRad),
        1.0e-5,
        "d(Delta H dot)/d(Delta Theta)"
    );
}

/// Проверяет отсутствие зависимости правой части от горизонтальной координаты.
void testDownrangeDoesNotAffectDynamics() {
    const auto dynamics = makeDynamics();
    const auto matrix = dynamics.linearize(
        makeNominalState()
    );

    const std::size_t downrangeColumn = indexOf(
        passive_flight::LongitudinalStateIndex::Downrange
    );

    for (std::size_t row = 0;
         row < passive_flight::kLongitudinalStateDimension;
         ++row) {
        checkNear(
            matrix[row][downrangeColumn],
            0.0,
            0.0,
            "Dynamics is invariant to absolute downrange"
        );
    }
}

/// Проверяет знак статической восстанавливающей реакции по углу атаки.
void testStaticRestoringResponse() {
    const auto dynamics = makeDynamics();

    passive_flight::LongitudinalPerturbationState
        perturbation;

    perturbation.pitchAngleRad =
        0.1 *
        std::numbers::pi_v<double> /
        180.0;

    const auto evaluation =
        dynamics.evaluate(
            makeNominalState(),
            perturbation
        );

    check(
        evaluation.derivative.pitchRateRadps2 < 0.0,
        "Positive Delta alpha produces a restoring "
        "pitch acceleration"
    );
}

/// Сверяет `A*Delta x` с конечной разностью нелинейной правой части.
void testLinearPredictionMatchesNonlinearDifference() {
    const auto dynamics = makeDynamics();
    const auto nominalState = makeNominalState();

    passive_flight::LongitudinalPerturbationState
        perturbation;

    perturbation.speedMps = 1.0e-3;
    perturbation.flightPathAngleRad = 1.0e-7;
    perturbation.pitchRateRadps = -1.0e-7;
    perturbation.pitchAngleRad = 2.0e-7;
    perturbation.altitudeM = 1.0e-3;

    const auto linearEvaluation =
        dynamics.evaluate(
            nominalState,
            perturbation
        );

    auto perturbedState = nominalState;
    perturbedState.speedMps += perturbation.speedMps;
    perturbedState.flightPathAngleRad +=
        perturbation.flightPathAngleRad;
    perturbedState.pitchRateRadps +=
        perturbation.pitchRateRadps;
    perturbedState.pitchAngleRad +=
        perturbation.pitchAngleRad;
    perturbedState.altitudeM +=
        perturbation.altitudeM;

    const auto nominalDerivative =
        dynamics.nominalDynamics()
            .evaluate(nominalState)
            .derivative;

    const auto perturbedDerivative =
        dynamics.nominalDynamics()
            .evaluate(perturbedState)
            .derivative;

    checkNear(
        linearEvaluation.derivative.speedMps2,
        perturbedDerivative.speedMps2 -
            nominalDerivative.speedMps2,
        1.0e-8,
        "Linear Delta V-dot prediction"
    );

    checkNear(
        linearEvaluation.derivative
            .flightPathAngleRadps,
        perturbedDerivative.flightPathAngleRadps -
            nominalDerivative.flightPathAngleRadps,
        1.0e-10,
        "Linear Delta Theta-dot prediction"
    );

    checkNear(
        linearEvaluation.derivative.pitchRateRadps2,
        perturbedDerivative.pitchRateRadps2 -
            nominalDerivative.pitchRateRadps2,
        1.0e-8,
        "Linear Delta omega-z-dot prediction"
    );
}

/// Проверяет отклонение некорректных шагов и не конечных возмущений.
void testInvalidOptionsAndPerturbationAreRejected() {
    const auto passport =
        passive_flight::makeFab1500TPostnikovPassport();

    passive_flight::PerturbationLinearizationOptions
        invalidOptions;

    invalidOptions.angleStepRad = 0.0;

    checkThrows<std::invalid_argument>(
        [&]() {
            const passive_flight::
                PerturbedLongitudinalDynamics dynamics(
                    passport.object,
                    invalidOptions
                );

            static_cast<void>(dynamics);
        },
        "Zero differentiation step must be rejected"
    );

    const auto dynamics = makeDynamics();

    passive_flight::LongitudinalPerturbationState
        invalidPerturbation;

    invalidPerturbation.speedMps =
        std::numeric_limits<double>::quiet_NaN();

    checkThrows<std::invalid_argument>(
        [&]() {
            static_cast<void>(
                dynamics.evaluate(
                    makeNominalState(),
                    invalidPerturbation
                )
            );
        },
        "Non-finite perturbation must be rejected"
    );
}

} // namespace

/// Запускает проверки численной линеаризации продольной динамики.
int main() {
    testZeroPerturbationHasZeroDerivative();
    testKinematicMatrixTerms();
    testDownrangeDoesNotAffectDynamics();
    testStaticRestoringResponse();
    testLinearPredictionMatchesNonlinearDifference();
    testInvalidOptionsAndPerturbationAreRejected();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " perturbed longitudinal dynamics test(s) failed\n";

        return 1;
    }

    std::cout
        << "All perturbed longitudinal dynamics tests passed\n";

    return 0;
}
