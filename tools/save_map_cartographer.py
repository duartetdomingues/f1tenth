import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import ReadMetrics , WriteState
import csv
from datetime import datetime
import os
import shutil
import subprocess

class CartographerMetricsLogger(Node):
    def __init__(self):
        super().__init__('cartographer_metrics_logger')
        self.cli = self.create_client(ReadMetrics, '/read_metrics')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('A esperar por /read_metrics...')

        self.req = ReadMetrics.Request()
        
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.out_dir = f"map_{timestamp}"
        
        self.call_service()
        self.save_map()

    def call_service(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.process_metrics(future.result())
        else:
            self.get_logger().error('Erro ao chamar /read_metrics')
            
    def save_map(self, filename="map_output.pbstream"):
        cli = self.create_client(WriteState, '/write_state')
        while not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('A esperar por /write_state...')
            
        os.makedirs(self.out_dir, exist_ok=True)
        
        filename = os.path.join(self.out_dir, filename)
        
        pbstream_path = os.path.abspath(filename)

        req = WriteState.Request()
        req.filename = pbstream_path
        
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Mapa guardado com sucesso em: {filename}")
        else:
            self.get_logger().error("Erro ao chamar /write_state para guardar o mapa")
            
        

            
        try:
            map_filestem = os.path.join(self.out_dir, "map_output")
            subprocess.run([
                "ros2", "run", "cartographer_ros", "cartographer_pbstream_to_ros_map",
                "-pbstream_filename", pbstream_path,
                "-map_filestem", map_filestem,
                "-resolution", "0.005"
            ], check=True)
            self.get_logger().info(f"Mapa convertido com sucesso para .pgm e .yaml em: {self.out_dir}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Erro ao converter pbstream: {e}")


    def process_metrics(self, response):
        # Inicialização de métricas
        constraints_found = constraints_searched = 0
        latency = None
        submaps = {'active': 0, 'frozen': 0, 'deleted': 0}
        
        
        MatchingResult = None
        InsertionResult = None
        work_queue_delay = None
        cpu_ratio = None
        real_time_ratio = None
        num_submap_scan_matchers = None

        constraints_score_histogram = {}
        residuals_histogram = {}
        costs_histogram = {}
        local_scores_histogram = {}
        constraint_scores_histogram = {}

        for family in response.metric_families:
            name = family.name

            if name == 'mapping_constraints_constraint_builder_2d_constraints':
                for m in family.metrics:
                    label_dict = {label.key: label.value for label in m.labels}
                    if label_dict.get('matcher') == 'found':
                        constraints_found += m.value
                    elif label_dict.get('matcher') == 'searched':
                        constraints_searched += m.value

            elif name == 'mapping_2d_local_trajectory_builder_latency':
                latency = family.metrics[0].value

            elif name == 'mapping_2d_pose_graph_submaps':
                for m in family.metrics:
                    label_dict = {label.key: label.value for label in m.labels}
                    if label_dict.get('state') == 'active':
                        submaps['active'] += m.value
                    elif label_dict.get('state') == 'frozen':
                        submaps['frozen'] += m.value
                    elif label_dict.get('state') == 'deleted':
                        submaps['deleted'] += m.value

            elif name == 'mapping_global_trajectory_builder_local_slam_results':
                for m in family.metrics:
                    label_dict = {label.key: label.value for label in m.labels}
                    if label_dict.get('type') == 'MatchingResult':
                        MatchingResult = m.value
                    elif label_dict.get('type') == 'InsertionResult':
                        InsertionResult = m.value
                                
           
                                
            

            elif name == 'mapping_constraints_constraint_builder_2d_num_submap_scan_matchers':
                num_submap_scan_matchers = family.metrics[0].value

            elif name == 'mapping_2d_pose_graph_work_queue_delay':
                work_queue_delay = family.metrics[0].value
                
            elif name == 'mapping_2d_local_trajectory_builder_real_time_ratio':
                real_time_ratio = family.metrics[0].value

            elif name == 'mapping_2d_local_trajectory_builder_cpu_real_time_ratio':
                cpu_ratio = family.metrics[0].value
                
            elif name == 'mapping_constraints_constraint_builder_2d_scores':
                for m in family.metrics:
                    if m.counts_by_bucket:
                        for bucket in m.counts_by_bucket:
                            if bucket.count > 0:
                                boundary = round(bucket.bucket_boundary, 2)
                                constraints_score_histogram[boundary] = constraints_score_histogram.get(boundary, 0) + bucket.count


            elif name == 'mapping_2d_local_trajectory_builder_residuals':
                for m in family.metrics:
                    if m.counts_by_bucket:
                        for bucket in m.counts_by_bucket:
                            if bucket.count > 0:
                                boundary = round(bucket.bucket_boundary, 2)
                                residuals_histogram[boundary] = residuals_histogram.get(boundary, 0) + bucket.count

            elif name == 'mapping_2d_local_trajectory_builder_costs':
                for m in family.metrics:
                    if m.counts_by_bucket:
                        for bucket in m.counts_by_bucket:
                            if bucket.count > 0:
                                boundary = round(bucket.bucket_boundary, 2)
                                costs_histogram[boundary] = costs_histogram.get(boundary, 0) + bucket.count

            elif name == 'mapping_2d_local_trajectory_builder_scores':
                for m in family.metrics:
                    if m.counts_by_bucket:
                        for bucket in m.counts_by_bucket:
                            if bucket.count > 0:
                                boundary = round(bucket.bucket_boundary, 2)
                                local_scores_histogram[boundary] = local_scores_histogram.get(boundary, 0) + bucket.count

        matching_rate = (constraints_found / constraints_searched) if constraints_searched else 0.0

        # Create a directory with the current timestamp
        os.makedirs(self.out_dir, exist_ok=True)

        # Copy the .lua files into the directory
        lua_files = [
            "/home/f1tenth/src/cartographer_ros/cartographer_ros/configuration_files/backpack_2d.lua",
            "/home/f1tenth/src/cartographer/configuration_files/trajectory_builder_2d.lua"
        ]
        for lua_file in lua_files:
            if os.path.exists(lua_file):
                shutil.copy(lua_file, self.out_dir)

        # Save the CSV file in the directory
        filename = os.path.join(self.out_dir, "cartographer_metrics.csv")
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Metric', 'Value'])
            writer.writerow(['Constraints Found', constraints_found])
            writer.writerow(['Constraints Searched', constraints_searched])
            writer.writerow(['MatchingResult Score', MatchingResult])
            writer.writerow(['InsertionResult Score', InsertionResult])
            writer.writerow(['Matching Rate (%)', round(matching_rate * 100, 2)])
            writer.writerow(['Latency (s)', latency])
            writer.writerow(['Submaps:', submaps])
            writer.writerow(['Num Submap Scan Matchers', num_submap_scan_matchers])
            writer.writerow(['Work Queue Delay (s)', work_queue_delay])
            writer.writerow(['Real-time Ratio (Sensor / Wall clock duration)', real_time_ratio])
            writer.writerow(['CPU Real-time Ratio (Sensor / Cpu duration)', cpu_ratio])

            writer.writerow([])
            writer.writerow(['Matching Score Bucket (Bucket , Count)'])
            for boundary in sorted(constraint_scores_histogram):
                writer.writerow([boundary, constraint_scores_histogram[boundary]])

            writer.writerow([])
            writer.writerow(['Local Trajectory Builder Residuals (Bucket , Count)'])
            for boundary in sorted(residuals_histogram):
                writer.writerow([boundary, residuals_histogram[boundary]])

            writer.writerow([])
            writer.writerow(['Local Trajectory Builder Costs (Bucket , Count)'])
            for boundary in sorted(costs_histogram):
                writer.writerow([boundary, costs_histogram[boundary]])

            writer.writerow([])
            writer.writerow(['Local Trajectory Builder Scores (Bucket , Count)'])
            for boundary in sorted(local_scores_histogram):
                writer.writerow([boundary, local_scores_histogram[boundary]])

        self.get_logger().info(f"Métricas guardadas em: {filename}")


def main():
    rclpy.init()
    logger = CartographerMetricsLogger()
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()