#pragma once

#include <vector>

namespace passive_flight {

/**
 * Одна точка произвольной аэродинамической зависимости
 * от числа Маха.
 */
struct MachCoefficientPoint {
    /// Число Маха узла таблицы, безразмерное.
    double mach{0.0};

    /// Значение коэффициента или его производной в данном узле;
    /// единица определяется конкретной таблицей.
    double value{0.0};
};

/**
 * Полный набор готовых табличных аэродинамических
 * характеристик пассивного объекта.
 *
 * Для ФАБ-1500Т используются:
 *
 *     Cx0(M);
 *     Cx^(alpha^2)(M);
 *     Cy^alpha(M);
 *     mz^alpha(M);
 *     mz^(omegaBar)(M).
 */
struct TabulatedAerodynamicData {
    /// Коэффициент сопротивления при нулевом угле атаки `Cx0(M)`, безразмерный.
    std::vector<MachCoefficientPoint> cx0;

    /// Коэффициент квадратичной зависимости сопротивления
    /// `Cx^(alpha²)(M)`, 1/рад².
    std::vector<MachCoefficientPoint> cxAlphaSquared;

    /// Производная коэффициента нормальной силы `Cy^alpha(M)`, 1/рад.
    std::vector<MachCoefficientPoint> cyAlpha;

    /// Производная статического коэффициента момента `mz^alpha(M)`, 1/рад.
    std::vector<MachCoefficientPoint> mzAlpha;

    /// Производная `mz` по безразмерной скорости
    /// `omega_bar_z = omega_z*L_ref/V`, безразмерная.
    std::vector<MachCoefficientPoint> mzPitchRate;
};

} // namespace passive_flight
