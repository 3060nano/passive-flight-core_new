#include "passive_flight/CubicSpline.hpp"

#include <algorithm>
#include <stdexcept>

namespace passive_flight {

void CubicSpline::build(
    const std::vector<double>& x,
    const std::vector<double>& y
) {
    if (x.size() != y.size() || x.size() < 2) {
        throw std::invalid_argument(
            "CubicSpline requires at least two points"
        );
    }

    x_ = x;
    y_ = y;

    const int n = static_cast<int>(x_.size()) - 1;

    a_ = y_;
    b_.assign(n, 0.0);
    c_.assign(n + 1, 0.0);
    d_.assign(n, 0.0);

    std::vector<double> h(n);
    std::vector<double> alpha(n);
    std::vector<double> l(n + 1);
    std::vector<double> mu(n + 1);
    std::vector<double> z(n + 1);

    for (int i = 0; i < n; ++i) {
        h[i] = x_[i + 1] - x_[i];
        if (h[i] <= 0.0) {
            throw std::invalid_argument(
                "CubicSpline abscissae must be strictly increasing"
            );
        }
    }

    for (int i = 1; i < n; ++i) {
        alpha[i] =
            3.0 / h[i] * (a_[i + 1] - a_[i]) -
            3.0 / h[i - 1] * (a_[i] - a_[i - 1]);
    }

    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (int i = 1; i < n; ++i) {
        l[i] = 2.0 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n] = 1.0;
    z[n] = 0.0;
    c_[n] = 0.0;

    for (int j = n - 1; j >= 0; --j) {
        c_[j] = z[j] - mu[j] * c_[j + 1];
        b_[j] =
            (a_[j + 1] - a_[j]) / h[j] -
            h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
        d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
    }
}

double CubicSpline::evaluate(double x) const {
    if (x_.empty()) {
        throw std::logic_error("CubicSpline is not built");
    }

    if (x <= x_.front()) {
        return y_.front();
    }

    if (x >= x_.back()) {
        return y_.back();
    }

    const auto it = std::upper_bound(x_.begin(), x_.end(), x);
    const int i = static_cast<int>(it - x_.begin()) - 1;
    const double dx = x - x_[i];

    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
}

} // namespace passive_flight
