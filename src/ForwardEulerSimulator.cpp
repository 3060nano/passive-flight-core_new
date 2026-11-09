#include "passive_flight/ForwardEulerSimulator.hpp"

#include "passive_flight/ModelContract.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace passive_flight {
namespace {

/// Проверяет конечность всех компонент состояния траектории.
/// @param state Время и компоненты состояния в принятых единицах СИ.
/// @return `true`, если каждая компонента представима конечным числом.
bool isFiniteState(
    const State& state
) {
    return
        std::isfinite(state.timeS) &&
        std::isfinite(state.speedMps) &&
        std::isfinite(state.flightPathAngleRad) &&
        std::isfinite(state.pitchRateRadps) &&
        std::isfinite(state.pitchAngleRad) &&
        std::isfinite(state.downrangeM) &&
        std::isfinite(state.altitudeM);
}

/// Проверяет численные настройки явного интегрирования.
/// @param options Шаг и предельное время в с, высота поверхности в м,
/// предельные счётчики шагов и частоты сохранения.
/// @return `true`, если настройки конечны и лежат в допустимых диапазонах.
bool areValidOptions(
    const SimulationOptions& options
) {
    return
        std::isfinite(options.timeStepS) &&
        options.timeStepS > 0.0 &&

        std::isfinite(options.maximumTimeS) &&
        options.maximumTimeS > 0.0 &&

        options.maximumSteps > 0 &&

        std::isfinite(options.groundAltitudeM) &&

        options.historyStride > 0;
}

/// Проверяет соответствие запроса фиксированной модели решателя.
/// @param request Идентификатор модели, высота сброса в м и скорость в м/с.
/// @param options Настройки, содержащие высоту поверхности в м.
/// @param object Фиксированная числовая модель расчётного объекта.
/// @return `true`, если идентификатор совпадает и условия сброса допустимы.
bool isValidRequestForObject(
    const SimulationRequest& request,
    const SimulationOptions& options,
    const ObjectModel& object
) {
    if (request.objectId.empty()) {
        return false;
    }

    /*
     * Идентификатор является полем ObjectModel,
     * а не ObjectMetadata.
     */
    if (request.objectId != object.id) {
        return false;
    }

    if (!std::isfinite(request.release.altitudeM) ||
        !std::isfinite(request.release.speedMps)) {
        return false;
    }

    if (request.release.speedMps <= 0.0) {
        return false;
    }

    if (request.release.altitudeM <=
        options.groundAltitudeM) {
        return false;
    }

    return true;
}

/// Формирует диагностические величины для одного состояния траектории.
/// В частности, вычисляет число Рейнольдса
/// `Re = rho*V*D/mu`, где `D` — диаметр корпуса, м.
/// @param state Состояние объекта в единицах СИ.
/// @param evaluation Результат вычисления правой части в том же состоянии.
/// @param object Модель с диаметром корпуса и опорной геометрией в СИ.
/// @return Диагностика с безразмерными числами, силами в Н и моментом в Н·м.
StepDiagnostics makeStepDiagnostics(
    const State& state,
    const LongitudinalDynamicsEvaluation& evaluation,
    const ObjectModel& object
) {
    StepDiagnostics diagnostics;

    diagnostics.angleOfAttackRad =
        evaluation.angleOfAttackRad;

    diagnostics.angleOfAttackRateRadps =
        evaluation.angleOfAttackRateRadS;

    diagnostics.mach =
        evaluation.mach;

    /*
     * Число Рейнольдса:
     *
     * Re = rho * V * b_A / mu.
     */
    diagnostics.reynolds =
        evaluation.atmosphere.densityKgM3 *
        state.speedMps *
        object.body.diameterM /
        evaluation.atmosphere.dynamicViscosityPaS;

    diagnostics.dynamicPressurePa =
        evaluation.dynamicPressurePa;

    diagnostics.dragCoefficient =
        evaluation.aerodynamics.cx;

    diagnostics.liftCoefficient =
        evaluation.aerodynamics.cy;

    diagnostics.pitchingMomentCoefficient =
        evaluation.aerodynamics.mz;

    diagnostics.dragN =
        evaluation.loads.dragN;

    diagnostics.liftN =
        evaluation.loads.normalForceN;

    diagnostics.pitchingMomentNm =
        evaluation.loads.pitchingMomentNm;

    return diagnostics;
}

/// Объединяет состояние и соответствующую ему диагностику в точку истории.
/// @param state Состояние траектории в единицах СИ.
/// @param evaluation Правая часть и нагрузки для этого состояния.
/// @param object Модель объекта, необходимая для расчёта числа Рейнольдса.
/// @return Полностью заполненная точка истории траектории.
TrajectorySample makeTrajectorySample(
    const State& state,
    const LongitudinalDynamicsEvaluation& evaluation,
    const ObjectModel& object
) {
    TrajectorySample sample;

    sample.state = state;

    sample.diagnostics =
        makeStepDiagnostics(
            state,
            evaluation,
            object
        );

    return sample;
}

/// Добавляет точку истории либо заменяет точку с тем же моментом времени.
/// @param result Изменяемый результат моделирования.
/// @param sample Новая точка с временем в с и состоянием в единицах СИ.
void appendSampleIfDifferent(
    SimulationResult& result,
    const TrajectorySample& sample
) {
    if (result.history.empty()) {
        result.history.push_back(sample);
        return;
    }

    const double previousTimeS =
        result.history.back().state.timeS;

    if (std::abs(
            previousTimeS -
            sample.state.timeS
        ) > 1.0e-12) {
        result.history.push_back(sample);
    } else {
        result.history.back() = sample;
    }
}

/// Выполняет один шаг явного метода Эйлера
/// `x_(n+1) = x_n + dt*f(x_n)` для всех компонент состояния.
/// @param current Состояние в начале шага в единицах СИ.
/// @param derivative Правая часть системы в согласованных единицах СИ.
/// @param timeStepS Длительность шага `dt`, с.
/// @return Состояние на конце шага в единицах СИ.
State makeEulerStep(
    const State& current,
    const StateDerivative& derivative,
    double timeStepS
) {
    State next;

    next.timeS =
        current.timeS +
        timeStepS;

    next.speedMps =
        current.speedMps +
        timeStepS *
        derivative.speedMps2;

    next.flightPathAngleRad =
        current.flightPathAngleRad +
        timeStepS *
        derivative.flightPathAngleRadps;

    next.pitchRateRadps =
        current.pitchRateRadps +
        timeStepS *
        derivative.pitchRateRadps2;

    next.pitchAngleRad =
        current.pitchAngleRad +
        timeStepS *
        derivative.pitchAngleRadps;

    next.downrangeM =
        current.downrangeM +
        timeStepS *
        derivative.downrangeMps;

    next.altitudeM =
        current.altitudeM +
        timeStepS *
        derivative.altitudeMps;

    return next;
}

/// Вычисляет линейную интерполяцию `y = y0 + lambda*(y1-y0)`.
/// @param first Значение при `lambda = 0` в единицах интерполируемой величины.
/// @param second Значение при `lambda = 1` в тех же единицах.
/// @param interpolationParameter Безразмерный параметр `lambda`.
/// @return Интерполированное значение в единицах аргументов first и second.
double interpolateValue(
    double first,
    double second,
    double interpolationParameter
) {
    return
        first +
        interpolationParameter *
        (second - first);
}

/// Уточняет состояние пересечения заданной горизонтальной поверхности.
/// Каждая компонента линейно интерполируется между состояниями по разные
/// стороны поверхности; высота результата точно равна groundAltitudeM.
/// @param aboveGround Состояние над поверхностью в единицах СИ.
/// @param belowGround Состояние под поверхностью в единицах СИ.
/// @param groundAltitudeM Геометрическая высота поверхности, м.
/// @return Интерполированное состояние события достижения поверхности.
State interpolateGroundImpact(
    const State& aboveGround,
    const State& belowGround,
    double groundAltitudeM
) {
    const double altitudeDifference =
        aboveGround.altitudeM -
        belowGround.altitudeM;

    double interpolationParameter = 1.0;

    if (std::abs(altitudeDifference) > 1.0e-12) {
        interpolationParameter =
            (
                aboveGround.altitudeM -
                groundAltitudeM
            ) /
            altitudeDifference;
    }

    interpolationParameter =
        std::clamp(
            interpolationParameter,
            0.0,
            1.0
        );

    State impact;

    impact.timeS =
        interpolateValue(
            aboveGround.timeS,
            belowGround.timeS,
            interpolationParameter
        );

    impact.speedMps =
        interpolateValue(
            aboveGround.speedMps,
            belowGround.speedMps,
            interpolationParameter
        );

    impact.flightPathAngleRad =
        interpolateValue(
            aboveGround.flightPathAngleRad,
            belowGround.flightPathAngleRad,
            interpolationParameter
        );

    impact.pitchRateRadps =
        interpolateValue(
            aboveGround.pitchRateRadps,
            belowGround.pitchRateRadps,
            interpolationParameter
        );

    impact.pitchAngleRad =
        interpolateValue(
            aboveGround.pitchAngleRad,
            belowGround.pitchAngleRad,
            interpolationParameter
        );

    impact.downrangeM =
        interpolateValue(
            aboveGround.downrangeM,
            belowGround.downrangeM,
            interpolationParameter
        );

    impact.altitudeM =
        groundAltitudeM;

    return impact;
}

} // namespace

ForwardEulerSimulator::ForwardEulerSimulator(
    ObjectModel object
)
    : object_(std::move(object)),
      dynamics_(object_) {
}

SimulationResult ForwardEulerSimulator::simulate(
    const SimulationRequest& request,
    const SimulationOptions& options
) const {
    SimulationResult result;

    if (!areValidOptions(options) ||
        !isValidRequestForObject(
            request,
            options,
            object_
        )) {
        result.terminationReason =
            TerminationReason::InvalidInput;

        return result;
    }

    State current =
        makeHorizontalReleaseState(
            request.release
        );

    if (!isFiniteState(current)) {
        result.terminationReason =
            TerminationReason::InvalidInput;

        result.finalState = current;
        return result;
    }

    try {
        const auto initialEvaluation =
            dynamics_.evaluate(current);

        if (options.saveHistory) {
            appendSampleIfDifferent(
                result,
                makeTrajectorySample(
                    current,
                    initialEvaluation,
                    object_
                )
            );
        }
    } catch (...) {
        result.terminationReason =
            TerminationReason::InvalidState;

        result.finalState = current;
        return result;
    }

    std::size_t completedSteps = 0;

    while (true) {
        if (current.altitudeM <=
            options.groundAltitudeM) {
            current.altitudeM =
                options.groundAltitudeM;

            result.finalState = current;

            result.terminationReason =
                TerminationReason::GroundReached;

            return result;
        }

        if (current.timeS >=
            options.maximumTimeS) {
            result.finalState = current;

            result.terminationReason =
                TerminationReason::MaximumTimeReached;

            if (options.saveHistory) {
                try {
                    appendSampleIfDifferent(
                        result,
                        makeTrajectorySample(
                            current,
                            dynamics_.evaluate(current),
                            object_
                        )
                    );
                } catch (...) {
                    result.terminationReason =
                        TerminationReason::InvalidState;
                }
            }

            return result;
        }

        if (completedSteps >=
            options.maximumSteps) {
            result.finalState = current;

            result.terminationReason =
                TerminationReason::MaximumStepsReached;

            if (options.saveHistory) {
                try {
                    appendSampleIfDifferent(
                        result,
                        makeTrajectorySample(
                            current,
                            dynamics_.evaluate(current),
                            object_
                        )
                    );
                } catch (...) {
                    result.terminationReason =
                        TerminationReason::InvalidState;
                }
            }

            return result;
        }

        const double remainingTimeS =
            options.maximumTimeS -
            current.timeS;

        const double actualTimeStepS =
            std::min(
                options.timeStepS,
                remainingTimeS
            );

        if (actualTimeStepS <= 0.0) {
            result.finalState = current;

            result.terminationReason =
                TerminationReason::MaximumTimeReached;

            return result;
        }

        LongitudinalDynamicsEvaluation evaluation;

        try {
            evaluation =
                dynamics_.evaluate(current);
        } catch (...) {
            result.finalState = current;

            result.terminationReason =
                TerminationReason::InvalidState;

            return result;
        }

        State next =
            makeEulerStep(
                current,
                evaluation.derivative,
                actualTimeStepS
            );

        ++completedSteps;

        if (!isFiniteState(next) ||
            next.speedMps <= 0.0) {
            result.finalState = current;

            result.terminationReason =
                TerminationReason::InvalidState;

            return result;
        }

        if (next.altitudeM <=
            options.groundAltitudeM) {
            State impact =
                interpolateGroundImpact(
                    current,
                    next,
                    options.groundAltitudeM
                );

            if (!isFiniteState(impact) ||
                impact.speedMps <= 0.0) {
                result.finalState = current;

                result.terminationReason =
                    TerminationReason::InvalidState;

                return result;
            }

            result.finalState = impact;
            result.terminationReason =
                TerminationReason::GroundReached;

            if (options.saveHistory) {
                try {
                    const auto impactEvaluation =
                        dynamics_.evaluate(impact);

                    appendSampleIfDifferent(
                        result,
                        makeTrajectorySample(
                            impact,
                            impactEvaluation,
                            object_
                        )
                    );
                } catch (...) {
                    result.terminationReason =
                        TerminationReason::InvalidState;
                }
            }

            return result;
        }

        current = next;

        if (options.saveHistory &&
            completedSteps %
                options.historyStride == 0) {
            try {
                appendSampleIfDifferent(
                    result,
                    makeTrajectorySample(
                        current,
                        dynamics_.evaluate(current),
                        object_
                    )
                );
            } catch (...) {
                result.finalState = current;

                result.terminationReason =
                    TerminationReason::InvalidState;

                return result;
            }
        }
    }
}

const ObjectModel&
ForwardEulerSimulator::object() const noexcept {
    return object_;
}

const NominalLongitudinalDynamics&
ForwardEulerSimulator::dynamics() const noexcept {
    return dynamics_;
}

} // namespace passive_flight
