#pragma once

#include "passive_flight/NominalLongitudinalDynamics.hpp"
#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/Types.hpp"

#include <array>
#include <cstddef>

namespace passive_flight {

/// Число компонент продольного вектора состояния, безразмерный счётчик.
inline constexpr std::size_t
    kLongitudinalStateDimension = 6;

/// Индексы компонент продольного вектора состояния и матрицы Якоби.
enum class LongitudinalStateIndex : std::size_t {
    /// Модуль скорости `V`, м/с.
    Speed = 0,

    /// Угол наклона траектории `Theta`, рад.
    FlightPathAngle = 1,

    /// Угловая скорость тангажа `omega_z`, рад/с.
    PitchRate = 2,

    /// Угол тангажа `theta`, рад.
    PitchAngle = 3,

    /// Горизонтальная координата `x`, м.
    Downrange = 4,

    /// Геометрическая высота `H`, м.
    Altitude = 5
};

/// Шестикомпонентный вектор продольного состояния; единицы зависят от компоненты.
using LongitudinalStateVector =
    std::array<double, kLongitudinalStateDimension>;

/// Матрица Якоби `A = df/dx` размером 6x6; размерность каждого элемента
/// равна отношению единицы соответствующей производной к единице состояния.
using LongitudinalSystemMatrix =
    std::array<
        LongitudinalStateVector,
        kLongitudinalStateDimension
    >;

/**
 * Малые отклонения от невозмущённой траектории.
 *
 * Порядок компонент совпадает с State:
 *
 * [Delta V, Delta Theta, Delta omega_z,
 *  Delta theta, Delta x, Delta H].
 */
struct LongitudinalPerturbationState {
    /// Малое отклонение скорости `Delta V`, м/с.
    double speedMps{0.0};

    /// Малое отклонение угла траектории `Delta Theta`, рад.
    double flightPathAngleRad{0.0};

    /// Малое отклонение угловой скорости `Delta omega_z`, рад/с.
    double pitchRateRadps{0.0};

    /// Малое отклонение угла тангажа `Delta theta`, рад.
    double pitchAngleRad{0.0};

    /// Малое отклонение горизонтальной координаты `Delta x`, м.
    double downrangeM{0.0};

    /// Малое отклонение геометрической высоты `Delta H`, м.
    double altitudeM{0.0};
};

/**
 * Правая часть вариационного уравнения продольного движения
 * `d(Delta x)/dt = A(t) Delta x`.
 *
 * Каждое поле содержит производную соответствующей компоненты малого
 * отклонения по времени; физические единицы указаны отдельно для полей.
 */
struct LongitudinalPerturbationDerivative {
    /// Производная отклонения скорости `d(Delta V)/dt`, м/с².
    double speedMps2{0.0};

    /// Производная отклонения угла траектории, рад/с.
    double flightPathAngleRadps{0.0};

    /// Производная отклонения угловой скорости, рад/с².
    double pitchRateRadps2{0.0};

    /// Производная отклонения угла тангажа, рад/с.
    double pitchAngleRadps{0.0};

    /// Производная отклонения горизонтальной координаты, м/с.
    double downrangeMps{0.0};

    /// Производная отклонения высоты, м/с.
    double altitudeMps{0.0};
};

/**
 * Шаги численного дифференцирования правой части.
 *
 * Шаги являются размерными и задаются в СИ.
 */
struct PerturbationLinearizationOptions {
    /// Приращение скорости для численного дифференцирования, м/с.
    double speedStepMps{1.0e-3};

    /// Приращение углов Theta и theta, рад.
    double angleStepRad{1.0e-7};

    /// Приращение угловой скорости тангажа, рад/с.
    double pitchRateStepRadps{1.0e-7};

    /// Приращение горизонтальной координаты, м.
    double positionStepM{1.0e-3};

    /// Приращение геометрической высоты, м.
    double altitudeStepM{1.0e-3};
};

/// Результат вычисления линеаризованной правой части в текущей точке.
struct PerturbedLongitudinalEvaluation {
    /// Матрица Якоби нелинейной правой части вдоль номинальной траектории.
    LongitudinalSystemMatrix systemMatrix{};

    /// Производная возмущения, вычисленная как `d(Delta x)/dt = A*Delta x`.
    LongitudinalPerturbationDerivative derivative{};
};

/**
 * Линеаризованная система продольного возмущённого движения.
 *
 * Для текущего состояния невозмущённой траектории x*(t)
 * вычисляется матрица Якоби:
 *
 *     A(t) = df/dx | x*(t),
 *
 * после чего малое возмущение определяется системой:
 *
 *     Delta x dot = A(t) * Delta x.
 */
class PerturbedLongitudinalDynamics {
public:
    /// Создаёт линеаризатор для заданного объекта и шагов дифференцирования.
    /// @param object Числовая модель объекта в единицах СИ.
    /// @param options Положительные размерные шаги численного дифференцирования.
    /// @throws std::invalid_argument Если хотя бы один шаг некорректен.
    explicit PerturbedLongitudinalDynamics(
        ObjectModel object,
        PerturbationLinearizationOptions options = {}
    );

    /// Вычисляет матрицу Якоби `A = df/dx` нелинейной правой части.
    /// Внутри области определения применяется центральная разность
    /// `[f(x+h)-f(x-h)]/(2h)`; у границы — односторонняя разность.
    /// @param nominalState Точка номинальной траектории в единицах СИ.
    /// @return Матрица 6x6 с покомпонентными физическими размерностями.
    /// @throws std::runtime_error Если столбец нельзя вычислить ни одной схемой.
    [[nodiscard]]
    LongitudinalSystemMatrix linearize(
        const State& nominalState
    ) const;

    /// Вычисляет правую часть системы малых возмущений
    /// `d(Delta x)/dt = A(t)*Delta x`.
    /// @param nominalState Номинальное состояние `x*(t)` в единицах СИ.
    /// @param perturbation Малое отклонение `Delta x(t)` в единицах СИ.
    /// @return Матрица Якоби и производная малого отклонения.
    /// @throws std::invalid_argument Если компоненты возмущения не конечны.
    [[nodiscard]]
    PerturbedLongitudinalEvaluation evaluate(
        const State& nominalState,
        const LongitudinalPerturbationState& perturbation
    ) const;

    /// Возвращает нелинейную правую часть номинального движения.
    /// @return Константная ссылка, действующая в течение жизни объекта.
    [[nodiscard]]
    const NominalLongitudinalDynamics&
    nominalDynamics() const noexcept;

    /// Возвращает используемые шаги численного дифференцирования.
    /// @return Константная ссылка на размерные настройки линеаризации.
    [[nodiscard]]
    const PerturbationLinearizationOptions&
    options() const noexcept;

private:
    /// Нелинейная модель, относительно которой строится матрица Якоби.
    NominalLongitudinalDynamics nominalDynamics_;

    /// Размерные шаги конечных разностей для каждой компоненты состояния.
    PerturbationLinearizationOptions options_;
};

} // namespace passive_flight
