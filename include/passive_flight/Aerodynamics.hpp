#pragma once

namespace passive_flight {

/// Входные параметры табличной аэродинамической модели продольного движения.
struct AerodynamicInput {
    /// Число Маха `M = V/a`, безразмерное.
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
    /// Коэффициент сопротивления при нулевом угле атаки, безразмерный.
    double cx0{0.0};

    /// Углозависимая добавка `Cx^(alpha²)(M)*alpha²`, безразмерная.
    double cxInduced{0.0};

    /// Полный коэффициент сопротивления `Cx`, безразмерный.
    double cx{0.0};

    /// Полный коэффициент нормальной силы `Cy`, безразмерный.
    double cy{0.0};

    /// Статическая составляющая коэффициента момента `mz^alpha*alpha`, безразмерная.
    double mzStatic{0.0};

    /// Демпфирующая составляющая `mz^(omega_bar)*omega_bar_z`, безразмерная.
    double mzPitchDamping{0.0};

    /// Полный коэффициент продольного момента `mz`, безразмерный.
    double mz{0.0};

    /// Производная Cy по углу атаки, 1/рад.
    double cyAlphaPerRad{0.0};

    /// Производная статического mz по углу атаки, 1/рад.
    double mzAlphaPerRad{0.0};

    /// Производная `mz` по `omega_bar_z = omega_z*L_ref/V`, безразмерная.
    double mzPitchRateDerivative{0.0};
};

/// Общий контракт источника аэродинамических коэффициентов.
class AerodynamicModel {
public:
    /// Виртуальный деструктор общего полиморфного интерфейса.
    virtual ~AerodynamicModel() = default;

    /// Вычисляет коэффициенты сил и момента для заданного режима движения.
    /// Конкретная реализация обязана сохранять соглашения о знаках и нормировке.
    /// @param input Число Маха, угол атаки, скорость и угловая скорость в СИ.
    /// @return Безразмерные аэродинамические коэффициенты и их производные.
    [[nodiscard]]
    virtual AerodynamicCoefficients evaluate(
        const AerodynamicInput& input
    ) const = 0;
};

} // namespace passive_flight
