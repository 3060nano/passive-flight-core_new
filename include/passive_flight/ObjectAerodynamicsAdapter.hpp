#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/ObjectModel.hpp"

#include <memory>

namespace passive_flight {

/// Создаёт табличную аэродинамическую модель из числового паспорта объекта.
[[nodiscard]]
std::shared_ptr<const AerodynamicModel>
makeAerodynamicModel(
    const ObjectModel& object
);

} // namespace passive_flight
