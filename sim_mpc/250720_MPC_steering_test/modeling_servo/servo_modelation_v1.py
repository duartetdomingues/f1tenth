from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import Twist  # Exemplo de tipo de mensagem para yaw_rate
from ackermann_msgs.msg import AckermannDriveStamped  # Exemplo para o comando do servo
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from scipy.optimize import curve_fit
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import savemat




# Função para calcular tau
def calculate_tau(delta, yaw_rate, vel):
    # Parâmetro de ganho do servo (ajustável conforme as características do servo)
    # Exemplo de cálculo de tau usando um modelo de primeira ordem: tau = (delta - yaw_rate * L / v) / yaw_rate
    # Você precisa do comprimento do carro (L) e da velocidade (v)
    L = 0.35  # comprimento do carro em metros (ajuste conforme seu veículo)
    v = vel   # velocidade do carro em m/s (ajuste conforme seu experimento)

    if abs(yaw_rate) > 1e-6:
        tau = (delta - yaw_rate * L / v) / yaw_rate
    else:
        tau = 0.0  # Evita divisão por zero
    
    return tau

def estimate_delta(yaw_rate, vel):
    # Estimar delta com base na taxa de giro e velocidade
    # Exemplo simples: delta = yaw_rate * L / v
    L = 0.35  # comprimento do carro em metros (ajuste conforme seu veículo)

    if abs(vel) > 1e-6:
        delta = yaw_rate * L / vel
    else:
        delta = 0.0  # Evita divisão por zero
    
    return delta




# Função para ler mensagens do rosbag2
def read_ros2_bag(bag_path):
    # Inicialize o ROS 2
    rclpy.init()

    # Criação do nó para leitura
    node = Node('rosbag_reader')

    # Abrindo o arquivo rosbag2
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)
    
    # Get topic information
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    total_tau = 0.0  # Variável para acumular tau
    count = 0  # Contador para o número de mensagens processadas
    
    # Variables to store the latest values from different topics
    delta_cmd = []
    delta_cmd_t = []  # Lista para armazenar timestamps correspondentes
    estimated_deltas = []  # Lista para armazenar estimativas de delta
    estimated_deltas_t = []  # Lista para armazenar timestamps correspondentes
    yaw_rate = None
    vel = None
    
    # Iterating over the messages in the bag
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        
        # Deserialize the message
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        
        if topic == '/mpc/control_array':  # Supondo que o comando do servo esteja neste tópico
            # A mensagem AckermannDriveStamped contém o comando do servo
            delta_cmd.append(msg.data[0])  # Supondo que o comando do servo seja o primeiro elemento
            delta_cmd_t.append(timestamp)  # Armazenar o timestamp correspondente
            #print(f"Comando do Servo (delta_cmd): {delta_cmd}")

        elif topic == '/imu_base_link':  # Supondo que a taxa de giro da IMU esteja neste tópico
            # A mensagem de IMU contém o yaw_rate
            yaw_rate = msg.angular_velocity.z  # A taxa de giro em torno do eixo Z
            #print(f"Taxa de Giro (yaw_rate): {yaw_rate}")
            
        elif topic == '/odometry/filtered':  # Supondo que a taxa de giro da IMU esteja neste tópico
            # A mensagem de IMU contém o yaw_rate
            vel = msg.twist.twist.linear.x
            #print(f"Velocidade Linear (vel): {vel}")

        # Calculate tau only when we have all three values
        if yaw_rate is not None and vel is not None:
            # Calcular tau para o servo
            """ if abs(delta_cmd) > 0.1 and abs(yaw_rate) > 0.2 and vel > 0.5:  # Evita valores muito pequenos
                tau = calculate_tau(delta_cmd, yaw_rate, vel)
                total_tau += tau
                count += 1
                print(f"Constante de Tempo (tau): {tau}, delta_cmd: {delta_cmd}, yaw_rate: {yaw_rate}, vel: {vel}")
                delta_cmd = None
                yaw_rate = None
                vel = None """
                
            if vel > 0.3:  # Evita valores muito pequenos
                estimated_deltas.append(estimate_delta(yaw_rate, vel))  # Estimar delta com base na taxa de giro e velocidade
                estimated_deltas_t.append(timestamp)  # Armazenar o timestamp correspondente
                count += 1
                yaw_rate = None
                vel = None
                

    # Fechando o arquivo do rosbag
    # Note: SequentialReader doesn't have a close() method in recent versions

    # Finalizando o nó ROS 2
    rclpy.shutdown()
    
    # Exibir o resultado final
    if count > 0:
        print(f"Estimativas de Delta count: {len(estimated_deltas)}")
        #print(f"Constante de Tempo Média (tau): {average_tau} based on {count} mensagens processadas.")
    else:
        print("Nenhuma mensagem processada.")
        
    #fit estimated_deltas to delta_cmd
    
    plt.figure(figsize=(10, 5))
    plt.plot(np.array(delta_cmd_t) * 1e-9, delta_cmd, 'o-', label='Comando do Servo (delta_cmd)')
    plt.plot(np.array(estimated_deltas_t) * 1e-9, estimated_deltas, 'x-', label='Estimativa de Delta')
    plt.xlabel('Tempo [s]')
    plt.ylabel('Delta [rad]')
    plt.title('Comparação entre Comando do Servo e Estimativa de Delta')
    plt.legend()
    plt.grid()
    plt.show()
    
    # Interpolar delta_cmd para os timestamps de estimated_deltas
    delta_cmd_interp = np.interp(
        np.array(estimated_deltas_t) * 1e-9,
        np.array(delta_cmd_t) * 1e-9,
        delta_cmd
    )

    plt.figure(figsize=(10, 5))
    plt.plot(np.array(estimated_deltas_t) * 1e-9, delta_cmd_interp, 's-', label='Servo Cmd Interpolado')
    plt.plot(np.array(estimated_deltas_t) * 1e-9, estimated_deltas, 'x-', label='Estimativa de Delta')
    plt.xlabel('Tempo [s]')
    plt.ylabel('Delta [rad]')
    plt.title('Servo Cmd Interpolado vs Estimativa de Delta')
    plt.legend()
    plt.grid()
    plt.show()

    t = (np.array(estimated_deltas_t) - estimated_deltas_t[0] ) * 1e-9  # Convertendo timestamps para segundos
    
    if bag_path == '/home/desktop/Documents/rosbag/mpc_test_steering_07_22/bag_1.5m_s/rosbag2_2025_07_22-18_02_29_0.db3':
        t = t[t < 22]
        estimated_deltas = estimated_deltas[:len(t)]
        delta_cmd_interp = delta_cmd_interp[:len(t)]
        print(f"Reduzindo o tempo para 22 segundos, tamanho de t: {len(t)}, estimated_deltas: {len(estimated_deltas)}, delta_cmd_interp: {len(delta_cmd_interp)}")
    #
    # Função para o curve_fit
    def model_to_fit(t, tau):
        return simulate_first_order(delta_cmd_interp, t, tau, 1.0)

    def simulate_first_order(cmd, t, tau, gain):
        dt = t[1] - t[0]
        y = np.zeros_like(cmd)
        for k in range(1, len(cmd)):
            y[k] = y[k-1] + dt * (-(1 / tau) * y[k-1] + (gain / tau) * cmd[k-1])
        return y
    
    
    
    # Ajuste
    p0 = [0.3]  # palpite inicial para [tau, ganho]
    popt, _ = curve_fit(model_to_fit, t, estimated_deltas, p0=p0)

    tau_est= popt[0]
    gain_est = 1.0  # Ganho fixo para o ajuste
    print(f"Tau estimado: {tau_est:.4f} s, ganho estimado: {gain_est:.4f}")
    
    rosbag_dir = bag_path.rsplit('/', 1)[0]  # Obtém o diretório do rosbag
    
    df = pd.DataFrame({
    "tempo_s": t,
    "servo_estimado": estimated_deltas,
    "servo_cmd": delta_cmd_interp,
    "tau_estimado": tau_est,
    "ganho_estimado": gain_est
    })

    df.to_csv(f"{rosbag_dir}/dados_estimacao_tau.csv", index=False)

    np.savez(f"{rosbag_dir}/plot_data.npz", t=t, servo_cmd=delta_cmd_interp, servo_estimado=estimated_deltas, tau_estimado=tau_est, ganho_estimado=gain_est)

    # Arrays que queres guardar
    dados = {
        't': t,  # vetor de tempo
        "servo_estimado": estimated_deltas,
        "servo_cmd": delta_cmd_interp,
        "tau_estimado": tau_est,
        "ganho_estimado": gain_est
    }

    # Salvar num ficheiro .mat
    savemat(f"{rosbag_dir}/dados_estimacao.mat", dados)

    # Visualizar
    plt.plot(t, delta_cmd_interp, '--', label='Comando (cmd_servo)')
    plt.plot(t, estimated_deltas, label='Servo medido')
    plt.plot(t, model_to_fit(t, *popt), label='Ajuste 1ª ordem')
    plt.xlabel('Tempo [s]')
    plt.ylabel('Posição servo')
    plt.title('Estimativa de tau com entrada arbitrária')
    plt.grid(True)
    plt.legend()
    plt.show()

    

# Caminho para o arquivo rosbag2
#bag_path = '/home/desktop/Documents/rosbag/mpc_test_steering_07_22/bag_1m_s/rosbag2_2025_07_22-16_35_34_0.db3'
#bag_path = '/home/desktop/Documents/rosbag/mpc_test_steering_07_22/bag_1.2m_s/rosbag2_2025_07_22-17_33_39_0.db3'
bag_path = '/home/desktop/Documents/rosbag/mpc_test_steering_07_22/bag_1.5m_s/rosbag2_2025_07_22-18_02_29_0.db3'

# Chamar a função para ler o rosbag2 e calcular tau
read_ros2_bag(bag_path)
