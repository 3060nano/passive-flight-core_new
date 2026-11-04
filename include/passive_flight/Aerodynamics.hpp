#pragma once

namespace passive_flight {

/// Входные параметры табличной аэродинамической модели продольного движения.
struct AerodynamicInput {
    /// Число Маха.
    double mach{0.0};

    /// Угол атаки, рад.
    double angleOfAttackRad{0.0};

    /// Угловая скорость тангажа, рад/с.
    double pitchRateRadS{0.0};

    /// Модуль воздушной скорости, м/с.
    double speedMps{0.0};
};

/// Коэффициенты сил и продольного момента с диагностическими производными.
struct AerodynamicCoefficients {
    /// Коэффициент сопротивления при нулевом угле атаки.
    double cx0{0.0};

    /// Добавка Cx^(alpha^2) * alpha^2.
    double cxInduced{0.0};

    /// Полный коэффициент сопротивления.
    double cx{0.0};

    /// Полный коэффициент нормальной силы.
    double cy{0.0};

    /// Статическая составляющая коэффициента момента.
    double mzStatic{0.0};

    /// Составляющая коэффициента момента от угловой скорости тангажа.
    double mzPitchDamping{0.0};

    /// Полный коэффициент продольного момента.
    double mz{0.0};

    /// Производная Cy по углу атаки, 1/рад.
    double cyAlphaPerRad{0.0};

    /// Производная статического mz по углу атаки, 1/рад.
    double mzAlphaPerRad{0.0};

    /// Производная mz по omega_bar_z = omega_z * L_ref / V.
    double mzPitchRateDerivative{0.0};
};

/// Общий контракт источника аэродинамических коэффициентов.
class AerodynamicModel {
public:
    virtual ~AerodynamicModel() = default;

    /// Вычисляет коэффициенты для заданного режима движения.
    [[nodiscard]]
    virtual AerodynamicCoefficients evaluate(
        const AerodynamicInput& input
    ) const = 0;
};

} // namespace passive_flight
