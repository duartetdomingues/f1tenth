#include "utils/coordinate_transform.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace control_utils
{
namespace
{
constexpr std::size_t kSearchDim = 2;
constexpr std::size_t kMaxLeafSize = 10;
} // namespace

CoordinateTransformer::CoordinateTransformer() = default;

void CoordinateTransformer::set_reference_trajectory(const std::vector<double> &s_traj,
                                                     const std::vector<double> &x_traj,
                                                     const std::vector<double> &y_traj,
                                                     const std::vector<double> &theta_traj)
{
    if (s_traj.size() != x_traj.size() ||
        s_traj.size() != y_traj.size() ||
        s_traj.size() != theta_traj.size())
    {
        throw std::invalid_argument("Reference trajectory vectors must have the same size.");
    }

    s_traj_ = s_traj;
    x_traj_ = x_traj;
    y_traj_ = y_traj;
    theta_traj_ = theta_traj;

    cloud_.x_traj = x_traj_;
    cloud_.y_traj = y_traj_;

    track_length_ = s_traj_.empty() ? 0.0 : s_traj_.back();
    ds_ = (s_traj_.size() > 1) ? (s_traj_[1] - s_traj_[0]) : 0.0;

    build_kd_tree();
    initialized_ = !s_traj_.empty() && kd_tree_ != nullptr;
}

void CoordinateTransformer::build_kd_tree()
{
    if (cloud_.x_traj.empty())
    {
        kd_tree_.reset();
        return;
    }

    kd_tree_ = std::make_unique<KDTree>(kSearchDim, cloud_,
                                        nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeafSize));
    kd_tree_->buildIndex();
}

std::size_t CoordinateTransformer::wrap_index(long long raw_index) const
{
    if (s_traj_.empty())
    {
        throw std::runtime_error("Reference trajectory is empty.");
    }

    const long long size = static_cast<long long>(s_traj_.size());
    long long idx = raw_index % size;
    if (idx < 0)
    {
        idx += size;
    }
    return static_cast<std::size_t>(idx);
}

FrenetResult CoordinateTransformer::global_to_local_pose(double x, double y, double psi) const
{
    if (!initialized_ || !kd_tree_)
    {
        throw std::runtime_error("Coordinate transformer is not initialised.");
    }

    std::array<double, kSearchDim> query{{x, y}};
    KDTree::IndexType closest_idx_raw = 0;
    double distance_sq = 0.0;
    kd_tree_->knnSearch(query.data(), 1, &closest_idx_raw, &distance_sq);
    (void)distance_sq;

    const std::size_t closest_idx = static_cast<std::size_t>(closest_idx_raw);

    if (closest_idx >= s_traj_.size())
    {
        throw std::out_of_range("KD-tree returned an index outside the trajectory bounds.");
    }

    FrenetResult result;
    result.closest_index = closest_idx;
    result.s = s_traj_[closest_idx];
    result.x_ref = x_traj_[closest_idx];
    result.y_ref = y_traj_[closest_idx];
    result.theta_ref = theta_traj_[closest_idx];

    result.n = (y - result.y_ref) * std::cos(result.theta_ref) -
               (x - result.x_ref) * std::sin(result.theta_ref);

    result.u = std::remainder(psi - result.theta_ref, 2.0 * M_PI);

    return result;
}

void CoordinateTransformer::local_to_global_pose(double s_val, double n_val, double u_val,
                                                 double &x, double &y, double &psi,
                                                 double &X_s, double &Y_s, double &theta_s) const
{
    if (!initialized_)
    {
        throw std::runtime_error("Coordinate transformer is not initialised.");
    }
    if (s_traj_.empty())
    {
        throw std::runtime_error("Reference trajectory is empty.");
    }

    double s_wrapped = s_val;
    if (track_length_ > 0.0)
    {
        s_wrapped = std::fmod(s_val, track_length_);
        if (s_wrapped < 0.0)
        {
            s_wrapped += track_length_;
        }
    }

    std::size_t closest_idx = 0;
    if (ds_ > 0.0)
    {
        const long long raw_index = static_cast<long long>(std::llround(s_wrapped / ds_));
        closest_idx = wrap_index(raw_index);
    }
    else
    {
        double min_diff = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < s_traj_.size(); ++i)
        {
            const double diff = std::abs(s_traj_[i] - s_wrapped);
            if (diff < min_diff)
            {
                min_diff = diff;
                closest_idx = i;
            }
        }
    }

    X_s = x_traj_[closest_idx];
    Y_s = y_traj_[closest_idx];
    theta_s = theta_traj_[closest_idx];

    x = X_s - n_val * std::sin(theta_s);
    y = Y_s + n_val * std::cos(theta_s);
    psi = std::remainder(theta_s + u_val, 2.0 * M_PI);
}

void CoordinateTransformer::cumtrapz(const std::vector<double> &s_traj,
                                     const std::vector<double> &kappa_traj,
                                     std::vector<double> &theta_traj)
{
    if (s_traj.size() != kappa_traj.size())
    {
        throw std::invalid_argument("s_traj and kappa_traj must have the same length.");
    }

    theta_traj.assign(s_traj.size(), 0.0);

    for (std::size_t i = 1; i < s_traj.size(); ++i)
    {
        const double ds = s_traj[i] - s_traj[i - 1];
        theta_traj[i] = theta_traj[i - 1] + 0.5 * ds * (kappa_traj[i] + kappa_traj[i - 1]);
    }
}

} // namespace control_utils
