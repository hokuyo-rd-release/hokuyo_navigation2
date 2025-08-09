import rclpy
import sys
import rosbag2_py

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 reindex_bag.py <input_bag_path> <output_bag_path>")
        sys.exit(1)

    input_bag_path = sys.argv[1]
    output_bag_path = sys.argv[2]
    
    rclpy.init()

    # --- リーダーの設定 (元のrosbagを読み込む) ---
    reader = rosbag2_py.SequentialReader()
    storage_options_reader = rosbag2_py.StorageOptions(
        uri=input_bag_path,
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    print(f"Reading from bag: {input_bag_path}")
    try:
        reader.open(storage_options_reader, converter_options)
    except Exception as e:
        print(f"Failed to open input bag: {e}")
        rclpy.shutdown()
        return
        
    topic_types = reader.get_all_topics_and_types()
    
    # --- ライターの設定 (新しいrosbagを書き込む) ---
    writer = rosbag2_py.SequentialWriter()
    storage_options_writer = rosbag2_py.StorageOptions(
        uri=output_bag_path,
        storage_id='sqlite3'
    )
    print(f"Writing to new bag: {output_bag_path}")
    writer.open(storage_options_writer, converter_options)

    for topic_metadata in topic_types:
        writer.create_topic(topic_metadata)
    
    # --- メッセージの読み込みと書き込み ---
    print("Processing messages...")
    message_count = 0
    error_count = 0
    
    while reader.has_next():
        try:
            (topic_name, data, timestamp) = reader.read_next()
            # 読み込んだメッセージを新しいバッグに書き込む
            writer.write(topic_name, data, timestamp)
            message_count += 1
        except Exception as e:
            print(f"Skipping message due to error: {e}")
            error_count += 1
            # 破損したメッセージはスキップして次へ進む
            continue

    print(f"Processing finished.")
    print(f"Total messages processed: {message_count}")
    print(f"Total messages skipped due to error: {error_count}")
    
    del reader
    del writer
    rclpy.shutdown()

if __name__ == '__main__':
    main()