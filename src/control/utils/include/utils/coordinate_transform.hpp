#ifndef CONTROL_UTILS_COORDINATE_TRANSFORM_HPP
#define CONTROL_UTILS_COORDINATE_TRANSFORM_HPP

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include <nanoflann.hpp>

namespace control_utils
{

struct PointCloud
{
    std::vector<double> x_traj;
    std::vector<double> y_traj;

    inline std::size_t kdtree_get_point_count() const { return x_traj.size(); }

    inline double kdtree_get_pt(std::size_t idx, std::size_t dim) const
    {
        if (idx >= x_traj.size())
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return (dim == 0) ? x_traj[idx] : y_traj[idx];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX &) const { return false; }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2>;

struct FrenetResult
{
    double s = std::numeric_limits<double>::quiet_NaN();
    double n = std::numeric_limits<double>::quiet_NaN();
    double u = std::numeric_limits<double>::quiet_NaN();
    double x_ref = std::numeric_limits<double>::quiet_NaN();
    double y_ref = std::numeric_limits<double>::quiet_NaN();
    double theta_ref = std::numeric_limits<double>::quiet_NaN();
    std::size_t closest_index = 0;
};

class CoordinateTransformer
{
public:
    CoordinateTransformer();
    ~CoordinateTransformer() = default;

    CoordinateTransformer(const CoordinateTransformer &) = delete;
    CoordinateTransformer &operator=(const CoordinateTransformer &) = delete;
    CoordinateTransformer(CoordinateTransformer &&) = default;
    CoordinateTransformer &operator=(CoordinateTransformer &&) = default;

    void set_reference_trajectory(const std::vector<double> &s_traj,
                                  const std::vector<double> &x_traj,
                                  const std::vector<double> &y_traj,
                                  const std::vector<double> &theta_traj);

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }
    [[nodiscard]] double track_length() const noexcept { return track_length_; }
    [[nodiscard]] double sample_length() const noexcept { return ds_; }

    FrenetResult global_to_local_pose(double x, double y, double psi) const;

    void local_to_global_pose(double s_val, double n_val, double u_val,
                              double &x, double &y, double &psi,
                              double &X_s, double &Y_s, double &theta_s) const;

    static void cumtrapz(const std::vector<double> &s_traj,
                         const std::vector<double> &kappa_traj,
                         std::vector<double> &theta_traj);

private:
    void build_kd_tree();
    std::size_t wrap_index(long long raw_index) const;

    std::vector<double> s_traj_;
    std::vector<double> x_traj_;
    std::vector<double> y_traj_;
    std::vector<double> theta_traj_;

    PointCloud cloud_;
    std::unique_ptr<KDTree> kd_tree_;

    double track_length_ = 0.0;
    double ds_ = 0.0;
    bool initialized_ = false;
};

} // namespace control_utils

#endif // CONTROL_UTILS_COORDINATE_TRANSFORM_HPP
