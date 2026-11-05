#pragma once

#include "passive_flight/NominalLongitudinalDynamics.hpp"
#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/Types.hpp"

namespace passive_flight {

    /**
     * Решатель системы невозмущённого продольного движения
     * явным методом Эйлера.
     *
     * Внешними параметрами являются:
     *
     * - идентификатор объекта;
     * - высота сброса;
     * - скорость сброса.
     *
     * Начальные углы и угловая скорость принимаются равными нулю.
     */
    class ForwardEulerSimulator {
    public:
        /// Создаёт решатель для одной фиксированной числовой модели объекта.
        /// @param object Модель с параметрами в СИ; значение перемещается в решатель.
        /// @throws std::invalid_argument Если параметры модели некорректны.
        explicit ForwardEulerSimulator(
            ObjectModel object
        );

        /**
         * Выполняет полный расчёт траектории.
         *
         * Метод не выбрасывает исключения из-за некорректного
         * запроса или состояния во время интегрирования.
         * Причина остановки возвращается в SimulationResult.
         *
         * Для каждого шага применяется явная формула Эйлера
         * `x_(n+1) = x_n + dt*f(x_n)`. Пересечение поверхности уточняется
         * линейной интерполяцией между последним состоянием над землёй и первым
         * состоянием под землёй.
         *
         * @param request Идентификатор объекта, высота в м и скорость в м/с.
         * @param options Шаг и пределы интегрирования, а также режим истории.
         * @return Результат с конечным состоянием, причиной остановки и историей.
         */
        [[nodiscard]]
        SimulationResult simulate(
            const SimulationRequest& request,
            const SimulationOptions& options = {}
        ) const;

        /// Возвращает фиксированную модель объекта данного решателя.
        /// @return Константная ссылка, действующая в течение жизни решателя.
        [[nodiscard]]
        const ObjectModel& object() const noexcept;

        /// Возвращает правую часть невозмущённого движения.
        /// @return Константная ссылка на NominalLongitudinalDynamics.
        [[nodiscard]]
        const NominalLongitudinalDynamics&
        dynamics() const noexcept;

    private:
        /// Числовая модель единственного объекта, обслуживаемого решателем.
        ObjectModel object_;

        /// Правая часть нелинейной системы невозмущённого движения.
        NominalLongitudinalDynamics dynamics_;
    };

} // namespace passive_flight
