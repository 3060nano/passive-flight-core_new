#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/Atmosphere.hpp"
#include "passive_flight/ObjectAerodynamicsAdapter.hpp"
#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/Types.hpp"

#include <memory>

namespace passive_flight {

/**
 * Размерные аэродинамические силы и момент.
 *
 * Знаки определяются принятой системой уравнений:
 *
 * dragN — положительный модуль силы сопротивления,
 * направленной против вектора скорости;
 *
 * normalForceN — нормальная сила Y, положительная
 * в сторону увеличения угла наклона траектории;
 *
 * pitchingMomentNm — продольный момент Mz.
 */
struct LongitudinalAerodynamicLoads {
    /// Положительный модуль силы сопротивления `X = q*S*Cx`, Н.
    double dragN{0.0};

    /// Нормальная сила `Y = q*S*Cy`, положительная к росту Theta, Н.
    double normalForceN{0.0};

    /// Продольный момент `Mz = q*S*L_ref*mz`, Н·м.
    double pitchingMomentNm{0.0};
};

/**
 * Полный результат вычисления правых частей системы.
 *
 * Помимо производных состояния здесь сохраняются
 * промежуточные значения.
 */
struct LongitudinalDynamicsEvaluation {
    /// Правая часть системы уравнений состояния в единицах СИ.
    StateDerivative derivative;

    /// Параметры атмосферы на текущей геометрической высоте.
    AtmosphereState atmosphere;

    /// Безразмерные аэродинамические коэффициенты текущего режима.
    AerodynamicCoefficients aerodynamics;

    /// Размерные аэродинамические силы и момент.
    LongitudinalAerodynamicLoads loads;

    /// Число Маха `M = V/a`, безразмерное.
    double mach{0.0};

    /// Скоростной напор `q = rho*V²/2`, Па.
    double dynamicPressurePa{0.0};

    /// Угол атаки `alpha = theta - Theta`, рад.
    double angleOfAttackRad{0.0};

    /// Производная угла атаки `d(alpha)/dt = omega_z - dTheta/dt`, рад/с.
    double angleOfAttackRateRadS{0.0};
};

/**
 * Правая часть нелинейной системы уравнений
 * невозмущённого продольного движения.
 *
 * Двигательная установка отсутствует.
 * Тяга принимается равной нулю.
 *
 * Класс зависит только от общего контракта
 * AerodynamicModel и не знает, каким способом
 * получены Cx, Cy и mz.
 */
class NominalLongitudinalDynamics {
public:
    /// Создаёт правую часть движения со стандартной атмосферой проекта.
    /// @param object Числовая модель с массой в кг, инерцией в кг·м²,
    /// опорной геометрией в СИ и аэродинамическими таблицами.
    /// @throws std::invalid_argument Если обязательные параметры некорректны.
    explicit NominalLongitudinalDynamics(
        ObjectModel object
    );

    /// Создаёт правую часть движения с явно заданной атмосферной моделью.
    /// @param object Числовая модель объекта в единицах СИ.
    /// @param atmosphere Валидированная модель атмосферы; значение копируется.
    /// @throws std::invalid_argument Если параметры объекта некорректны.
    NominalLongitudinalDynamics(
        ObjectModel object,
        StandardAtmosphere atmosphere
    );

    /**
     * Вычисляет производные состояния в заданной точке.
     *
     * Порядок компонент State:
     *
     * [V, Theta, omega_z, vartheta, x, H].
     *
     * Реализованы уравнения
     * `dV/dt = -X/m - g*sin(Theta)`,
     * `dTheta/dt = Y/(m*V) - g*cos(Theta)/V`,
     * `domega_z/dt = Mz/I_z`, `dtheta/dt = omega_z`,
     * `dx/dt = V*cos(Theta)` и `dH/dt = V*sin(Theta)`.
     *
     * @param state Состояние в единицах СИ, углы в радианах.
     * @return Производные состояния, атмосферу, коэффициенты и нагрузки.
     * @throws std::invalid_argument Если состояние некорректно.
     * @throws std::out_of_range Если высота или число Маха вне области моделей.
     */
    [[nodiscard]]
    LongitudinalDynamicsEvaluation evaluate(
        const State& state
    ) const;

    /// Возвращает числовую модель расчётного объекта.
    /// @return Константная ссылка, действующая в течение жизни динамической модели.
    [[nodiscard]]
    const ObjectModel& object() const noexcept;

    /// Возвращает используемую модель стандартной атмосферы.
    /// @return Константная ссылка на атмосферу и её параметры в СИ.
    [[nodiscard]]
    const StandardAtmosphere& atmosphere() const noexcept;

    /// Возвращает используемый источник аэродинамических коэффициентов.
    /// @return Полиморфная константная ссылка на AerodynamicModel.
    [[nodiscard]]
    const AerodynamicModel&
    aerodynamics() const noexcept;

private:
    /// Числовая модель объекта с массой, геометрией и аэродинамикой.
    ObjectModel object_;

    /// Атмосферная модель для вычисления `rho`, `a` и `mu` по высоте.
    StandardAtmosphere atmosphere_;

    /*
     * shared_ptr сохраняет копируемость
     * NominalLongitudinalDynamics и существующих
     * классов возмущённого движения.
     */
    /// Разделяемая неизменяемая реализация аэродинамического контракта.
    std::shared_ptr<const AerodynamicModel>
        aerodynamics_;
};

} // namespace passive_flight
