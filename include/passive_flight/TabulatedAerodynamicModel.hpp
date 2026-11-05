#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/CubicSpline.hpp"
#include "passive_flight/TabulatedAerodynamicData.hpp"

namespace passive_flight {

    /**
     * Аэродинамическая модель, использующая готовые
     * табличные характеристики объекта.
     *
     * Используются формулы:
     *
     *     Cx = Cx0(M) + Cx^(alpha^2)(M) * alpha^2;
     *
     *     Cy = Cy0(M) + Cy^alpha(M) * alpha;
     *
     *     mz = mz0(M) + mz^alpha(M) * alpha +
     *          mz^(omegaBar)(M) * omegaBar;
     *
     * где omegaBar = omega_z * L_ref / V.
     *
     * Таблицы Cy0 и mz0 необязательны (пустые ≡ 0).
     */
    class TabulatedAerodynamicModel final
        : public AerodynamicModel {
    public:
        TabulatedAerodynamicModel(
            double referenceLengthM,
            TabulatedAerodynamicData data
        );

        [[nodiscard]]
        AerodynamicCoefficients evaluate(
            const AerodynamicInput& input
        ) const override;

        [[nodiscard]]
        double referenceLengthM() const noexcept;

        [[nodiscard]]
        const TabulatedAerodynamicData& data() const noexcept;

    private:
        double referenceLengthM_{0.0};
        TabulatedAerodynamicData data_;

        CubicSpline cx0Spline_;
        CubicSpline cxAlphaSquaredSpline_;
        CubicSpline cy0Spline_;
        CubicSpline cyAlphaSpline_;
        CubicSpline mz0Spline_;
        CubicSpline mzAlphaSpline_;
        CubicSpline mzPitchRateSpline_;
    };

} // namespace passive_flight
