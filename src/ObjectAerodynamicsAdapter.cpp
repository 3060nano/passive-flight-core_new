#include "passive_flight/ObjectAerodynamicsAdapter.hpp"

#include "passive_flight/TabulatedAerodynamicModel.hpp"

#include <memory>
#include <stdexcept>

namespace passive_flight {

std::shared_ptr<const AerodynamicModel>
makeAerodynamicModel(
    const ObjectModel& object
) {
    if (object.aerodynamicModelType !=
        AerodynamicModelType::Tabulated) {
        throw std::invalid_argument(
            "Unsupported aerodynamic model type"
        );
    }

    return std::make_shared<TabulatedAerodynamicModel>(
        object.reference.effectiveReferenceLengthM(),
        object.tabulatedAerodynamics
    );
}

} // namespace passive_flight
