import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Criar um publicador de transformações TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Assinar o tópico de odometria
        self.odom_sub = self.create_subscription(
            Odometry,
            'vesc/odom',  # Altere para o nome correto do seu tópico
            self.odom_callback,
            10  # Frequência de callback
        )

    def odom_callback(self, msg):
        # Criar a mensagem de transformação
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom_vesc"  # Frame de origem
        t.child_frame_id = "base_link"  # Frame de destino

        # Preencher a posição e a orientação da odometria
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation  # Copiar a orientação do Odometry

        # Publicar a transformação
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
