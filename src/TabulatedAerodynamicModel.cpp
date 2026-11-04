#include "passive_flight/TabulatedAerodynamicModel.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace passive_flight {
namespace {

void requireFinite(double value, const char* name) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(std::string(name) + " must be finite");
    }
}

void validateTable(
    const std::vector<MachCoefficientPoint>& table,
    const char* name
) {
    if (table.size() < 2) {
        throw std::invalid_argument(
            std::string(name) + " table must contain at least two points"
        );
    }

    double previousMach = 0.0;
    bool first = true;

    for (const MachCoefficientPoint& point : table) {
        requireFinite(point.mach, "Mach");
        requireFinite(point.value, name);

        if (point.mach < 0.0) {
            throw std::invalid_argument(
                std::string(name) + " table contains negative Mach"
            );
        }

        if (!first && point.mach <= previousMach) {
            throw std::invalid_argument(
                std::string(name) +
                " table Mach values must be strictly increasing"
            );
        }

        previousMach = point.mach;
        first = false;
    }
}

void buildSpline(
    const std::vector<MachCoefficientPoint>& table,
    CubicSpline& spline
) {
    std::vector<double> mach;
    std::vector<double> values;
    mach.reserve(table.size());
    values.reserve(table.size());

    for (const MachCoefficientPoint& point : table) {
        mach.push_back(point.mach);
        values.push_back(point.value);
    }

    spline.build(mach, values);
}

double interpolate(
    const std::vector<MachCoefficientPoint>& table,
    const CubicSpline& spline,
    double mach,
    const char* name
) {
    if (mach < table.front().mach || mach > table.back().mach) {
        throw std::out_of_range(
            std::string(name) + " table does not cover requested Mach"
        );
    }

    return spline.evaluate(mach);
}

} // namespace

TabulatedAerodynamicModel::TabulatedAerodynamicModel(
    double referenceLengthM,
    TabulatedAerodynamicData data
)
    : referenceLengthM_(referenceLengthM),
      data_(std::move(data)) {
    if (!std::isfinite(referenceLengthM_) || referenceLengthM_ <= 0.0) {
        throw std::invalid_argument(
            "Reference length must be finite and positive"
        );
    }

    validateTable(data_.cx0, "Cx0");
    validateTable(data_.cxAlphaSquared, "CxAlphaSquared");
    validateTable(data_.cyAlpha, "CyAlpha");
    validateTable(data_.mzAlpha, "MzAlpha");
    validateTable(data_.mzPitchRate, "MzPitchRate");

    buildSpline(data_.cx0, cx0Spline_);
    buildSpline(data_.cxAlphaSquared, cxAlphaSquaredSpline_);
    buildSpline(data_.cyAlpha, cyAlphaSpline_);
    buildSpline(data_.mzAlpha, mzAlphaSpline_);
    buildSpline(data_.mzPitchRate, mzPitchRateSpline_);
}

AerodynamicCoefficients TabulatedAerodynamicModel::evaluate(
    const AerodynamicInput& input
) const {
    requireFinite(input.mach, "Mach");
    requireFinite(input.angleOfAttackRad, "Angle of attack");
    requireFinite(input.pitchRateRadS, "Pitch rate");
    requireFinite(input.speedMps, "Speed");

    if (input.mach < 0.0) {
        throw std::invalid_argument("Mach must be non-negative");
    }
    if (input.speedMps <= 0.0) {
        throw std::invalid_argument("Speed must be positive");
    }

    const double cx0 = interpolate(
        data_.cx0, cx0Spline_, input.mach, "Cx0"
    );
    const double cxAlphaSquared = interpolate(
        data_.cxAlphaSquared,
        cxAlphaSquaredSpline_,
        input.mach,
        "CxAlphaSquared"
    );
    const double cyAlpha = interpolate(
        data_.cyAlpha, cyAlphaSpline_, input.mach, "CyAlpha"
    );
    const double mzAlpha = interpolate(
        data_.mzAlpha, mzAlphaSpline_, input.mach, "MzAlpha"
    );
    const double mzPitchRateDerivative = interpolate(
        data_.mzPitchRate,
        mzPitchRateSpline_,
        input.mach,
        "MzPitchRate"
    );

    const double alpha = input.angleOfAttackRad;
    const double cxAngleDependent = cxAlphaSquared * alpha * alpha;
    const double cy = cyAlpha * alpha;
    const double mzStatic = mzAlpha * alpha;

    /*
     * Для таблиц ФАБ-1500Т используется соглашение источника:
     * omega_bar_z = omega_z * L_ref / V, без дополнительного множителя 1/2.
     */
    const double omegaBar =
        input.pitchRateRadS * referenceLengthM_ / input.speedMps;
    const double mzPitchDamping =
        mzPitchRateDerivative * omegaBar;

    AerodynamicCoefficients result;
    result.cx0 = cx0;
    result.cxInduced = cxAngleDependent;
    result.cx = cx0 + cxAngleDependent;
    result.cy = cy;
    result.mzStatic = mzStatic;
    result.mzPitchDamping = mzPitchDamping;
    result.mz = mzStatic + mzPitchDamping;
    result.cyAlphaPerRad = cyAlpha;
    result.mzAlphaPerRad = mzAlpha;
    result.mzPitchRateDerivative = mzPitchRateDerivative;
    return result;
}

double TabulatedAerodynamicModel::referenceLengthM() const noexcept {
    return referenceLengthM_;
}

const TabulatedAerodynamicData&
TabulatedAerodynamicModel::data() const noexcept {
    return data_;
}

} // namespace passive_flight
