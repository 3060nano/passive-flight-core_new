#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/CubicSpline.hpp"
#include "passive_flight/TabulatedAerodynamicData.hpp"

namespace passive_flight {

    /**
     * Аэродинамическая модель, использующая готовые
     * табличные характеристики объекта.
     *
     * Это реализация варианта №1:
     *
     *     полный паспорт объекта
     *     -> готовые аэродинамические коэффициенты
     *     -> движение.
     *
     * Используются формулы:
     *
     *     Cx =
     *         Cx0(M) +
     *         Cx^(alpha^2)(M) * alpha^2;
     *
     *     Cy =
     *         Cy^alpha(M) * alpha;
     *
     *     mz =
     *         mz^alpha(M) * alpha +
     *         mz^(omegaBar)(M) * omegaBar;
     *
     * где в принятом для табличных данных ФАБ-1500Т соглашении источника:
     *
     *     omegaBar = omega_z * L_ref / V.
     */
    class TabulatedAerodynamicModel final
        : public AerodynamicModel {
    public:
        /// Строит интерполяционные зависимости по пяти аэродинамическим таблицам.
        /// @param referenceLengthM Опорная длина в нормировке
        /// `omega_bar_z = omega_z*L_ref/V`, м.
        /// @param data Табличные коэффициенты и производные как функции числа Маха.
        /// @throws std::invalid_argument Если длина или таблицы некорректны.
        TabulatedAerodynamicModel(
            double referenceLengthM,
            TabulatedAerodynamicData data
        );

        /// Вычисляет коэффициенты по уравнениям
        /// `Cx = Cx0 + Cx^(alpha²)*alpha²`, `Cy = Cy^alpha*alpha` и
        /// `mz = mz^alpha*alpha + mz^(omega_bar)*omega_z*L_ref/V`.
        /// @param input Режим движения: число Маха безразмерно, углы в радианах,
        /// угловая скорость в рад/с и скорость в м/с.
        /// @return Безразмерные коэффициенты сил и момента с производными.
        /// @throws std::invalid_argument Если входные величины некорректны.
        /// @throws std::out_of_range Если число Маха вне общей области таблиц.
        [[nodiscard]]
        AerodynamicCoefficients evaluate(
            const AerodynamicInput& input
        ) const override;

        /// Возвращает опорную длину нормировки коэффициента момента.
        /// @return `L_ref`, м.
        [[nodiscard]]
        double referenceLengthM() const noexcept;

        /// Возвращает исходные табличные аэродинамические данные.
        /// @return Константная ссылка, действующая в течение жизни модели.
        [[nodiscard]]
        const TabulatedAerodynamicData& data() const noexcept;

    private:
        /// Опорная длина `L_ref` коэффициента момента, м.
        double referenceLengthM_{0.0};

        /// Исходные табличные коэффициенты и производные.
        TabulatedAerodynamicData data_;

        /// Сплайн зависимости `Cx0(M)`.
        CubicSpline cx0Spline_;

        /// Сплайн зависимости `Cx^(alpha²)(M)`, 1/рад².
        CubicSpline cxAlphaSquaredSpline_;

        /// Сплайн зависимости `Cy^alpha(M)`, 1/рад.
        CubicSpline cyAlphaSpline_;

        /// Сплайн зависимости `mz^alpha(M)`, 1/рад.
        CubicSpline mzAlphaSpline_;

        /// Сплайн зависимости `mz^(omega_bar)(M)`.
        CubicSpline mzPitchRateSpline_;
    };

} // namespace passive_flight
