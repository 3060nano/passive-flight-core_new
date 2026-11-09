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
        /// Строит интерполяционные зависимости аэродинамических коэффициентов.
        /// @param referenceLengthM Характерная длина `L_ref`, м, используемая
        /// для безразмерной скорости `omegaBar_z = omega_z*L_ref/V`.
        /// @param data Таблицы коэффициентов и производных по числу Маха.
        /// @throws std::invalid_argument Если длина или таблицы некорректны.
        TabulatedAerodynamicModel(
            double referenceLengthM,
            TabulatedAerodynamicData data
        );

        /// Вычисляет коэффициенты по интерполированным табличным зависимостям.
        /// Реализует уравнения `Cx = Cx0 + Cx^(alpha²)*alpha²`,
        /// `Cy = Cy0 + Cy^alpha*alpha` и
        /// `mz = mz0 + mz^alpha*alpha + mz^(omegaBar)*omegaBar_z`.
        /// @param input Число Маха, угол атаки в рад, скорость в м/с и
        /// угловая скорость тангажа в рад/с.
        /// @return Безразмерные коэффициенты и использованные производные.
        /// @throws std::invalid_argument Если входные значения некорректны.
        /// @throws std::out_of_range Если число Маха вне общей области таблиц.
        [[nodiscard]]
        AerodynamicCoefficients evaluate(
            const AerodynamicInput& input
        ) const override;

        /// Возвращает характерную длину нормировки продольного момента.
        /// @return Значение `L_ref`, м.
        [[nodiscard]]
        double referenceLengthM() const noexcept;

        /// Возвращает исходный набор аэродинамических таблиц.
        /// @return Константная ссылка, действующая в течение жизни модели.
        [[nodiscard]]
        const TabulatedAerodynamicData& data() const noexcept;

    private:
        /// Характерная длина `L_ref` для момента и `omegaBar_z`, м.
        double referenceLengthM_{0.0};

        /// Исходные таблицы коэффициентов и производных по числу Маха.
        TabulatedAerodynamicData data_;

        /// Сплайн безразмерного коэффициента `Cx0(M)`.
        CubicSpline cx0Spline_;

        /// Сплайн коэффициента `Cx^(alpha²)(M)`, рад⁻².
        CubicSpline cxAlphaSquaredSpline_;

        /// Сплайн безразмерной постоянной составляющей `Cy0(M)`.
        CubicSpline cy0Spline_;

        /// Сплайн производной `Cy^alpha(M)`, рад⁻¹.
        CubicSpline cyAlphaSpline_;

        /// Сплайн безразмерной постоянной составляющей `mz0(M)`.
        CubicSpline mz0Spline_;

        /// Сплайн производной `mz^alpha(M)`, рад⁻¹.
        CubicSpline mzAlphaSpline_;

        /// Сплайн производной `mz^(omegaBar)(M)`, безразмерной.
        CubicSpline mzPitchRateSpline_;
    };

} // namespace passive_flight
