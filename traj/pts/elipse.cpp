#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip> // std::setprecision
#include <sstream> // std::ostringstream

void create_elliptical_trajectory(double a, double b, double v_ref, size_t N)
{
    std::vector<double> x_traj(N), y_traj(N), v_traj(N, v_ref), yaw_traj(N);
    double delta_theta = 2 * M_PI / N;

    for (size_t i = 0; i < N; ++i)
    {
        double theta = i * delta_theta;
        x_traj[i] = a * cos(theta);
        y_traj[i] = -b - b * sin(theta);
    }

    // Calcular yaw (em radianos)
    for (size_t i = 0; i < N - 1; ++i)
    {
        double dx = x_traj[i + 1] - x_traj[i];
        double dy = y_traj[i + 1] - y_traj[i];
        yaw_traj[i] = atan2(dy, dx);
    }
    // Último yaw igual ao penúltimo
    yaw_traj[N - 1] = yaw_traj[N - 2];

    // Gerar nome de ficheiro com base nos parâmetros
    std::ostringstream filename_stream;
    filename_stream << "elipse_n" << N << "_a" << std::fixed << std::setprecision(1) << a
                    << "_b" << b << "_v" << v_ref << ".csv";
    std::string filename = filename_stream.str();

    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Erro ao abrir o ficheiro: " << filename << std::endl;
        return;
    }

    // Cabeçalho
    file << "x,y,yaw,v_ref\n";

    for (size_t i = 0; i < N; ++i)
    {
        file << x_traj[i] << "," << y_traj[i] << "," << yaw_traj[i] << "," << v_traj[i] << "\n";
    }

    file.close();
    std::cout << "Trajetória salva em: " << filename << std::endl;
}

int main()
{
    double raio_x = 2.5;
    double raio_y = 1.75;
    double velocidade_ref = 1.0;
    size_t N_pontos = 1000;

    create_elliptical_trajectory(raio_x, raio_y, velocidade_ref, N_pontos);

    return 0;
}
