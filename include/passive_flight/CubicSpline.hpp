#pragma once

#include <algorithm>
#include <vector>

namespace passive_flight {

/**
 * Естественный кубический сплайн.
 *
 * Используется для интерполяции аэродинамических
 * коэффициентов по числу Маха (аналог блока
 * «Кубическая сплайн-интерполяция» в SimInTech).
 *
 * Граничные условия: вторая производная на концах равна нулю.
 */
class CubicSpline {
public:
    CubicSpline() = default;

    void build(
        const std::vector<double>& x,
        const std::vector<double>& y
    );

    [[nodiscard]]
    double evaluate(double x) const;

    [[nodiscard]]
    bool empty() const noexcept {
        return x_.empty();
    }

private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> a_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;
};

} // namespace passive_flight
