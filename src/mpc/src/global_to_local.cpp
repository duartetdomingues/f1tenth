#include "mpc/global_to_local.hpp"

// Define the PointCloud structure for k-d tree
KDTreeWithCloud *prepare_kd_tree(const std::vector<double> &x_traj, const std::vector<double> &y_traj)
{
    PointCloud *cloud = new PointCloud();
    cloud->x_traj = x_traj;
    cloud->y_traj = y_traj;

    my_kd_tree_t *index = new my_kd_tree_t(2 /*dim*/, *cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index->buildIndex(); // Build the k-d tree once

    return new KDTreeWithCloud(cloud, index);
}

void global_to_local_pose(my_kd_tree_t *index, const std::vector<double> &s_traj,
                          const std::vector<double> &x_traj, const std::vector<double> &y_traj,
                          const std::vector<double> &theta_traj, double x, double y, double psi,
                          double &s_val, double &n_val, double &u_val, double &X_s, double &Y_s, double &theta_s)
{
    // Query the nearest neighbor (x, y) coordinates using the pre-built k-d tree
    std::vector<double> query_point = {x, y}; // Query point is a 2D point (x, y)
    size_t num_results = 1;                   // Only find the nearest neighbor
    unsigned int idx_query[1];                // Index of the nearest neighbor
    double dists_squared[1];                  // Squared distance of the nearest neighbor

    // Perform the search in the k-d tree (this searches over both x and y)
    index->knnSearch(&query_point[0], num_results, &idx_query[0], &dists_squared[0]);

    size_t closest_idx = idx_query[0]; // Closest index in trajectory
    s_val = s_traj[closest_idx];       // Closest s value (along the path)
    X_s = x_traj[closest_idx];
    Y_s = y_traj[closest_idx];
    theta_s = theta_traj[closest_idx]; // Orientation from theta_traj

    // Compute n_val (normal offset) from the global coordinates
    n_val = (x - X_s) * sin(theta_s) - (y - Y_s) * cos(theta_s);

    // Compute mu_val (heading difference) from the global orientation
    u_val = psi - theta_s;
}

// Função para calcular a integração acumulada de kappa para obter theta
void cumtrapz(const std::vector<double>& s_traj, const std::vector<double>& kappa_traj, std::vector<double>& theta_traj) {
    // Certifique-se de que as dimensões de s_traj e kappa_traj são iguais
    if (s_traj.size() != kappa_traj.size()) {
        std::cerr << "Erro: os vetores s_traj e kappa_traj devem ter o mesmo tamanho." << std::endl;
        return;
    }

    // Inicialize o vetor theta_traj com o mesmo tamanho de s_traj
    theta_traj.resize(s_traj.size(), 0.0);  // Inicializa com 0 (assumindo theta inicial = 0)

    // Método do trapézio para a integração acumulada
    for (size_t i = 1; i < s_traj.size(); ++i) {
        // Calcular a diferença entre os pontos de s_traj
        double ds = s_traj[i] - s_traj[i - 1];

        // Método do trapézio: soma da média dos kappa[i] e kappa[i-1], multiplicado pela diferença ds
        theta_traj[i] = theta_traj[i - 1] + 0.5 * ds * (kappa_traj[i] + kappa_traj[i - 1]);
    }
}

/* int main()
{
    // Example trajectory data (ensure they're properly initialized)
    std::vector<double> s_traj = {0.0, 1.0, 2.0, 3.0};     // Example s_traj
    std::vector<double> x_traj = {0.0, 1.0, 2.0, 3.0};     // Example x_traj
    std::vector<double> y_traj = {0.0, 1.0, 2.0, 3.0};     // Example y_traj
    std::vector<double> theta_traj = {0.0, 0.1, 0.2, 0.3}; // Example theta_traj

    // Prepare the k-d tree
    KDTreeWithCloud *kd_tree = prepare_kd_tree(x_traj, y_traj);

    // Example global coordinates and heading
    double x = 4.1;
    double y = 4.1;
    double psi = 0.15; // Example psi (orientation)

    // Initialize output variables
    double s_val, n_val, u_val, X_s, Y_s, theta_s;

    // Call the global_to_local_pose function
    global_to_local_pose(kd_tree->index, s_traj, x_traj, y_traj, theta_traj, x, y, psi, s_val, n_val, u_val, X_s, Y_s, theta_s);

    // Output the results
    std::cout << "s_val: " << s_val << ", n_val: " << n_val << ", u_val: " << u_val << std::endl;
    std::cout << "X_s: " << X_s << ", Y_s: " << Y_s << ", theta_s: " << theta_s << std::endl;

    // Clean up the allocated k-d tree memory
    delete kd_tree;

    return 0;
} */
