#ifndef GLOBAL_TO_LOCAL_HPP
#define GLOBAL_TO_LOCAL_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <nanoflann.hpp>

using namespace nanoflann;

// Define the PointCloud structure for k-d tree
struct PointCloud
{
    std::vector<double> x_traj, y_traj;

    // Nanoflann requires the following function to access data points
    inline size_t kdtree_get_point_count() const { return x_traj.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (idx >= x_traj.size())
        {
            std::cerr << "Error: Index out of bounds!" << std::endl;
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN if out of bounds
        }
        return (dim == 0) ? x_traj[idx] : y_traj[idx];
    }

    // Nanoflann requires this function for the k-d tree distance metric
    template <class BBOX>
    bool kdtree_get_bbox(BBOX &) const { return false; }
};

typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2> my_kd_tree_t;

// Function to build the k-d tree once
struct KDTreeWithCloud
{
    PointCloud *cloud;
    my_kd_tree_t *index;
    KDTreeWithCloud(PointCloud *c, my_kd_tree_t *i) : cloud(c), index(i) {}
    ~KDTreeWithCloud()
    {
        delete index;
        delete cloud;
    }
};

KDTreeWithCloud *prepare_kd_tree(const std::vector<double> &x_traj, const std::vector<double> &y_traj);

void global_to_local_pose(my_kd_tree_t *index, const std::vector<double> &s_traj,
                          const std::vector<double> &x_traj, const std::vector<double> &y_traj,
                          const std::vector<double> &theta_traj, double x, double y, double psi,
                          double &s_val, double &n_val, double &u_val, double &X_s, double &Y_s, double &theta_s);

void cumtrapz(const std::vector<double> &s_traj, const std::vector<double> &kappa_traj, std::vector<double> &theta_traj);

#endif // GLOBAL_TO_LOCAL_HPP