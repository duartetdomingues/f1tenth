import time
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from multiprocessing import Process, Queue
import os
import shutil

def writer_worker(output_bag, converter_options, included_topics, queue):
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=output_bag, storage_id='sqlite3'), converter_options)
    for topic in included_topics:
        writer.create_topic(topic)

    while True:
        item = queue.get()
        if item is None:
            break
        topic, data, t = item
        writer.write(topic, data, t)

def filter_bag_parallel(input_bag, output_bag, excluded_topics):
    start_time = time.time()
    excluded_set = set(excluded_topics)

    if os.path.exists(output_bag):
        print(f"[⚠️] O diretório {output_bag} já existe. Será removido.")
        shutil.rmtree(output_bag)

    storage_options = StorageOptions(uri=input_bag, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    included_topics = [t for t in topic_types if t.name not in excluded_set]

    queue = Queue(maxsize=1000)
    writer_process = Process(target=writer_worker, args=(output_bag, converter_options, included_topics, queue))
    writer_process.start()

    count_total = 0
    count_written = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        count_total += 1
        if topic not in excluded_set:
            queue.put((topic, data, t))
            count_written += 1

    queue.put(None)  # Signal end
    writer_process.join()

    duration = time.time() - start_time
    print(f"[✅] Bag filtrado salvo em: {output_bag}")
    print(f"[📊] Total de mensagens lidas: {count_total}")
    print(f"[📦] Mensagens escritas: {count_written}")
    print(f"[⏱️] Tempo total: {duration:.2f} segundos")

# === Uso ===
if __name__ == "__main__":
    input_bag_path = "/home/desktop/Documents/rosbag/rosbag2_2025_10_09-13_04_56"
    output_bag_path = input_bag_path + "_no_paths"
    topics_to_remove = [
        #"/zed2i/rgb/image_raw",
        "/odometry/filtered_path",
        "/vesc/odom_path"     
    ]
    filter_bag_parallel(input_bag_path, output_bag_path, topics_to_remove)
