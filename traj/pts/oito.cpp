#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip> // para std::setprecision
#include <sstream> // para std::ostringstream


void create_eight_trajectory(double a, double b, double v_ref, size_t N, const std::string& filename)
{
    std::vector<double> x_traj(N), y_traj(N), v_traj(N, v_ref), yaw_traj(N);

    double delta_t = 2 * M_PI / N;  // Para uma trajetória completa

    // Gerar os pontos da trajetória de 8, com (0, 0) na extremidade
    for (size_t i = 0; i < N; ++i)
    {
        double t = i * delta_t;
        x_traj[i] = a * sin(t);  // Posição x
        y_traj[i] = b * sin(t) * cos(t);  // Posição y
    }

    // Ajustar o ponto (0, 0) para uma das pontas da "oito"
    double offset_x = a;  // Translação para ajustar para uma das pontas
    double offset_y = 0.0;

    for (size_t i = 0; i < N; ++i)
    {
        x_traj[i] += offset_x;
        y_traj[i] += offset_y;
    }

    // Calcular o yaw (ângulo de orientação)
    for (size_t i = 0; i < N - 1; ++i)
    {
        double dx = x_traj[i + 1] - x_traj[i];
        double dy = y_traj[i + 1] - y_traj[i];
        yaw_traj[i] = atan2(dy, dx); // Calcula o ângulo de yaw
    }

    // O yaw do último ponto será igual ao penúltimo
    if (N > 1)
    {
        yaw_traj[N - 1] = yaw_traj[N - 2];
    }

    // Abrir o arquivo CSV para salvar os dados
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Erro ao abrir o ficheiro CSV: " << filename << std::endl;
        return;
    }

    // Cabeçalho do CSV
    file << "x,y,yaw,v_ref\n";

    // Escrever os dados no arquivo CSV
    for (size_t i = 0; i < N; ++i)
    {
        file << x_traj[i] << "," << y_traj[i] << "," << yaw_traj[i] << "," << v_traj[i] << "\n";
    }

    file.close();
    std::cout << "Trajetória salva em: " << filename << std::endl;
}

int main()
{
    double raio_x = 2.5;     // Raio X da "oito"
    double raio_y = 1.75;    // Raio Y da "oito"
    double velocidade_ref = 1.0;  // Velocidade de referência constante
    size_t N_pontos = 1000;  // Número de pontos

    // Gerar o nome do arquivo CSV
    std::ostringstream filename_stream;
    filename_stream << "oito_n" << N_pontos << "_a" << std::fixed << std::setprecision(1) << raio_x
                    << "_b" << raio_y << "_v" << velocidade_ref << ".csv";
    std::string filename = filename_stream.str();

    // Criar a trajetória de "oito" e salvar no arquivo CSV
    create_eight_trajectory(raio_x, raio_y, velocidade_ref, N_pontos, filename);

    return 0;
}
