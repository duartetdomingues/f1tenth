# This script analyzes a ROS 2 bag file to calculate the total size and average size of messages per topic.

import sqlite3
from collections import defaultdict
import os

# Get the path to the bag folder (not the metadata.db3 file)
bag_folder = '/home/desktop/Documents/rosbag/rosbag2_2025_10_09-12_59_26'
db_name = os.path.basename(bag_folder)
bag_path = os.path.join(bag_folder, db_name + '_0.db3')

print(f"Analyzing bag file at: {bag_path}")

conn = sqlite3.connect(bag_path)
cursor = conn.cursor()

# Obter mapping de topic_id para nome
cursor.execute("SELECT id, name FROM topics")
topic_map = {tid: name for tid, name in cursor.fetchall()}

# Contar mensagens e tamanho
sizes = defaultdict(int)
counts = defaultdict(int)

for row in cursor.execute("SELECT topic_id, LENGTH(data) FROM messages"):
    topic_id, size = row
    sizes[topic_id] += size
    counts[topic_id] += 1

# Mostrar resultados
for topic_id in sizes:
    name = topic_map[topic_id]
    total_mb = sizes[topic_id] / (1024*1024)
    avg_kb = (sizes[topic_id] / counts[topic_id]) / 1024
    print(f"{name:<50} | Total: {total_mb:6.2f} MB | Avg: {avg_kb:6.2f} KB/msg | Count: {counts[topic_id]}")
