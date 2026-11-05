#pragma once

#include "passive_flight/PerturbedTrajectorySimulator.hpp"
#include "passive_flight/Types.hpp"

namespace passive_flight {

/**
 * Изменения параметров падения, вызванные малым возмущением.
 *
 * Это не значения Delta x(t_f*) в момент падения номинальной
 * траектории, а изменения параметров самого события H = H_ground.
 *
 * Например:
 *
 *     fallTimeS = t_f - t_f*;
 *     downrangeM = L_f - L_f*.
 */
struct PerturbedImpactParameterChanges {
    /// Поправка к моменту достижения поверхности `Delta t_f`, с.
    double fallTimeS{0.0};

    /// Поправка к скорости в событии падения `Delta V_impact`, м/с.
    double speedMps{0.0};

    /// Поправка к углу траектории в событии падения, рад.
    double flightPathAngleRad{0.0};

    /// Поправка к угловой скорости тангажа в событии падения, рад/с.
    double pitchRateRadps{0.0};

    /// Поправка к углу тангажа в событии падения, рад.
    double pitchAngleRad{0.0};

    /// Поправка к горизонтальной дальности падения, м.
    double downrangeM{0.0};

    /// Вычисляет поправку к углу атаки как
    /// `Delta alpha = Delta theta - Delta Theta`.
    /// @return Поправка к углу атаки в событии падения, рад.
    [[nodiscard]]
    double angleOfAttackRad() const noexcept {
        return
            pitchAngleRad -
            flightPathAngleRad;
    }
};

/**
 * Результат первого линейного приближения параметров падения
 * возмущённого объекта.
 *
 * Номинальная траектория достигает поверхности в момент t_f*.
 * В этот момент малое возмущение по высоте в общем случае
 * не равно нулю:
 *
 *     Delta H_f = Delta H(t_f*).
 *
 * Из условия падения возмущённой траектории
 *
 *     H(t_f) = H_ground
 *
 * в первом порядке получается:
 *
 *     Delta t_f = -Delta H_f / H_dot_f*.
 *
 * Для любого другого параметра состояния y:
 *
 *     Delta y_impact =
 *         Delta y_f + y_dot_f* Delta t_f.
 */
struct PerturbedImpactAnalysis {
    /// Признак корректности линейной оценки параметров события падения.
    bool available{false};

    /**
     * Вертикальная скорость номинальной
     * траектории при падении `H_dot_f*`, м/с.
     */
    double nominalVerticalSpeedMps{0.0};

    /**
     * Изменения параметров самого события падения в единицах СИ.
     */
    PerturbedImpactParameterChanges changes;

    /**
     * Оценённое полное состояние возмущённого объекта
     * в его собственный момент достижения поверхности.
     *
     * Высота совпадает с высотой поверхности,
     * то есть с высотой конечного состояния
     * номинальной траектории; остальные компоненты заданы в единицах СИ.
     */
    State estimatedImpactState;
};

/**
 * Вычисляет первое линейное приближение изменения
 * параметров события падения H = H_ground.
 *
 * finalNominalDerivative должна быть правой частью
 * невозмущённой системы, вычисленной
 * в result.finalNominalState.
 *
 * Расчёт доступен только если номинальная траектория
 * завершилась по условию GroundReached и
 * H_dot_f* не близка к нулю.
 *
 * Реализованы уравнения первого порядка
 * `Delta t_f = -Delta H_f/H_dot_f*` и
 * `Delta y_impact = Delta y_f + y_dot_f*Delta t_f`.
 *
 * @param result Результат распространения малых возмущений до номинального падения.
 * @param finalNominalDerivative Правая часть номинальной системы в конечной точке;
 * компоненты заданы в единицах СИ.
 * @return Линейная оценка поправок и полного состояния падения; поле available
 * равно false, если исходные данные не позволяют выполнить оценку.
 */
[[nodiscard]]
PerturbedImpactAnalysis analyzePerturbedImpact(
    const PerturbedSimulationResult& result,
    const StateDerivative& finalNominalDerivative
);

} // namespace passive_flight
