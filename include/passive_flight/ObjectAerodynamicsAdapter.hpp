#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/ObjectModel.hpp"

#include <memory>

namespace passive_flight {

/// Создаёт полиморфную аэродинамическую модель из числового паспорта объекта.
/// @param object Валидированная модель с опорной длиной в метрах и таблицами.
/// @return Разделяемый неизменяемый объект AerodynamicModel.
/// @throws std::invalid_argument Если тип аэродинамики не поддерживается.
[[nodiscard]]
std::shared_ptr<const AerodynamicModel>
makeAerodynamicModel(
    const ObjectModel& object
);

} // namespace passive_flight
