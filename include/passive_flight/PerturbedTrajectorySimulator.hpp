#pragma once

#include "passive_flight/ForwardEulerSimulator.hpp"
#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/PerturbedLongitudinalDynamics.hpp"
#include "passive_flight/Types.hpp"

#include <vector>

namespace passive_flight {

/**
 * Одна точка совместной истории невозмущённого
 * и возмущённого движения.
 */
struct PerturbedTrajectorySample {
    /// Состояние невозмущённой траектории `x*(t)` в единицах СИ.
    State nominalState;

    /// Малое отклонение `Delta x(t)` в единицах СИ.
    LongitudinalPerturbationState perturbation;

    /// Линейная оценка полного состояния `x*(t) + Delta x(t)`.
    State totalState;
};

/**
 * Результат интегрирования малых возмущений
 * вдоль невозмущённой траектории.
 */
struct PerturbedSimulationResult {
    /// Причина остановки, наследуемая от номинального расчёта либо ошибка возмущений.
    TerminationReason terminationReason{
        TerminationReason::InvalidState
    };

    /// Конечное состояние невозмущённой траектории в единицах СИ.
    State finalNominalState;

    /// Конечный вектор малого отклонения в единицах СИ.
    LongitudinalPerturbationState finalPerturbation;

    /// Сумма конечного номинального состояния и малого отклонения.
    State finalTotalState;

    /// Совместная история номинального, возмущённого и полного состояний.
    std::vector<PerturbedTrajectorySample> history;
};

/**
 * Решатель линейного продольного возмущённого движения.
 *
 * Сначала рассчитывается невозмущённая траектория x*(t).
 * Затем по тем же временным узлам явным методом Эйлера
 * интегрируется система:
 *
 *     Delta x dot = A(t) * Delta x.
 *
 * Полное состояние для анализа формируется как:
 *
 *     x(t) = x*(t) + Delta x(t).
 */
class PerturbedTrajectorySimulator {
public:
    /// Создаёт совместный решатель номинального и линейного возмущённого движения.
    /// @param object Числовая модель объекта в единицах СИ.
    /// @param linearizationOptions Шаги конечных разностей в единицах СИ.
    explicit PerturbedTrajectorySimulator(
        ObjectModel object,
        PerturbationLinearizationOptions
            linearizationOptions = {}
    );

    /// Сначала рассчитывает номинальную траекторию, затем интегрирует вдоль неё
    /// систему `d(Delta x)/dt = A(t)*Delta x` явным методом Эйлера:
    /// `Delta x_(n+1) = Delta x_n + dt*A_n*Delta x_n`.
    /// @param request Идентификатор объекта и условия сброса в единицах СИ.
    /// @param initialPerturbation Начальное малое отклонение в единицах СИ.
    /// @param options Настройки шага, пределов и сохранения истории.
    /// @return Совместный результат номинального и возмущённого расчёта.
    [[nodiscard]]
    PerturbedSimulationResult simulate(
        const SimulationRequest& request,
        const LongitudinalPerturbationState&
            initialPerturbation,
        const SimulationOptions& options = {}
    ) const;

    /// Возвращает числовую модель расчётного объекта.
    /// @return Константная ссылка, действующая в течение жизни решателя.
    [[nodiscard]]
    const ObjectModel& object() const noexcept;

    /// Возвращает решатель невозмущённой траектории.
    /// @return Константная ссылка на ForwardEulerSimulator.
    [[nodiscard]]
    const ForwardEulerSimulator&
    nominalSimulator() const noexcept;

    /// Возвращает модель линеаризованной возмущённой динамики.
    /// @return Константная ссылка на PerturbedLongitudinalDynamics.
    [[nodiscard]]
    const PerturbedLongitudinalDynamics&
    perturbationDynamics() const noexcept;

private:
    /// Числовая модель объекта, общая для обоих решателей.
    ObjectModel object_;

    /// Решатель опорной невозмущённой траектории.
    ForwardEulerSimulator nominalSimulator_;

    /// Линеаризованная правая часть для распространения малых возмущений.
    PerturbedLongitudinalDynamics perturbationDynamics_;
};

} // namespace passive_flight
