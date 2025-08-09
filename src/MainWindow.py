#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import subprocess
import os
import glob

from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets

# ROS 2 の rosbag 関連ライブラリをインポート
try:
    from rosbag2_py import SequentialReader, SequentialWriter, StorageFilter, StorageOptions, ConverterOptions, TopicMetadata 
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy 
except ImportError:
    QtWidgets.QMessageBox.critical(None, "Error", "rosbag2_py or rclpy not found. Please ensure ROS 2 is sourced and required packages are installed.")
    sys.exit(1)


from FileWindow import FileWindow


class MainWindow(QMainWindow):
    def __init__(self, title="Select", msg="",app=None):
        super().__init__()

        self.msg = msg
        self._app = app

        self._files = []
        self._topic_list = []
        self._checkboxs=[]

        self.convert_btn=QPushButton("Convert")
        self.convert_btn.clicked.connect(self.on_convert)

        self.all_check_btn=QPushButton("All Check")
        self.all_check_btn.clicked.connect(self.on_all_check)

        self.all_uncheck_btn=QPushButton("All Uncheck")
        self.all_uncheck_btn.clicked.connect(self.on_all_uncheck)

        # when start up, user select file
        self.set_rosbag_topic()

        # window size and init position setting
        self.setGeometry(300, 300, 900, 600)

        # create menu
        self.create_menu_bar()

        self.setWindowTitle(title)
        self.show()
        
    def on_convert(self):
        """filter selecting topic (Self-implemented for ROS 2)
        """
        keeping_topics = [] # 選択されたトピック名のみを格納
        select_topic_count = False

        for (checkbox, select) in zip(self._checkboxs, self._topic_list):
            if checkbox.isChecked():
                keeping_topics.append(select)
                select_topic_count = True

        if select_topic_count is False:
            # not check topic
            QtWidgets.QMessageBox.critical(self, "Error", "Please select at least one topic")
            return

        output_path = FileWindow.save_file(isApp=True, caption="Convert bag file", filefilter="*.bag")
        if output_path[0] == "":
            return
        
        output_bag_folder_path = output_path[0]
        
        # ここから修正: 自動で '.bag' を付加するロジックを削除
        # if not (output_bag_folder_path.endswith('.bag') or \
        #         output_bag_folder_path.endswith('.db3') or \
        #         output_bag_folder_path.endswith('.mcap')):
        #     output_bag_folder_path += '.bag'
        # ここまで修正
        
        if os.path.exists(output_bag_folder_path) and \
           os.path.isdir(output_bag_folder_path) and \
           os.listdir(output_bag_folder_path):
            QtWidgets.QMessageBox.critical(self, "Error", 
                f"出力先のbagディレクトリ '{output_bag_folder_path}' は既に存在し、空ではありません。別の名前を選択するか、ディレクトリを空にしてください。")
            return

        input_bag_path = self._files[0]
        
        print(f"Reading from: {input_bag_path}")
        print(f"Writing to: {output_bag_folder_path}")
        print("Converting....")

        try:
            reader = SequentialReader()
            input_storage_id = 'sqlite3'
            
            if os.path.isdir(input_bag_path):
                input_uri_for_reader = input_bag_path
                if glob.glob(os.path.join(input_bag_path, '*.db3')):
                    input_storage_id = 'sqlite3'
                elif glob.glob(os.path.join(input_bag_path, '*.mcap')):
                    input_storage_id = 'mcap'
                else:
                    QtWidgets.QMessageBox.critical(self, "Error", f"No .db3 or .mcap files found in bag directory: {input_bag_path}")
                    return
            else:
                input_uri_for_reader = os.path.dirname(input_bag_path)
                if not input_uri_for_reader: input_uri_for_reader = "."
                
                if input_bag_path.endswith('.db3'):
                    input_storage_id = 'sqlite3'
                elif input_bag_path.endswith('.mcap'):
                    input_storage_id = 'mcap'
                elif input_bag_path.endswith('.bag'):
                    input_storage_id = 'sqlite3'
                else:
                    QtWidgets.QMessageBox.critical(self, "Error", f"Unsupported bag file extension for input: {input_bag_path}. Please select a .db3, .mcap, or .bag file.")
                    return

            storage_options_read = StorageOptions(uri=input_uri_for_reader, storage_id=input_storage_id)
            converter_options_read = ConverterOptions()
            reader.open(storage_options_read, converter_options_read)
            
            writer = SequentialWriter()
            storage_options_write = StorageOptions(uri=output_bag_folder_path, storage_id='sqlite3')
            converter_options_write = ConverterOptions()

            writer.open(storage_options_write, converter_options_write)
            
            topic_types_info = reader.get_all_topics_and_types()
            topic_names_to_types = {info.name: info.type for info in topic_types_info}

            topics_to_filter = set(keeping_topics)
            
            message_types = {}
            for topic_name in topics_to_filter:
                msg_type_str = topic_names_to_types.get(topic_name)
                if msg_type_str:
                    try:
                        msg_class = get_message(msg_type_str)
                        message_types[topic_name] = msg_class
                    except Exception as e:
                        print(f"Warning: Could not load message type {msg_type_str} for topic {topic_name}: {e}")
                        topics_to_filter.discard(topic_name)
                else:
                    print(f"Warning: No type found for topic {topic_name}. Skipping.")
                    topics_to_filter.discard(topic_name)

            # フィルタリングされたトピックを新しい bag ファイルに登録
            for topic_name in topics_to_filter:
                msg_type_str = topic_names_to_types[topic_name]

                # offered_qos_profiles に空文字列を渡すように修正 (前回修正済み)
                empty_qos_str = ""

                # TopicMetadata オブジェクトを作成
                topic_metadata = TopicMetadata(
                    name=topic_name,
                    type=msg_type_str,
                    serialization_format='cdr', # ROS 2 のデフォルトは 'cdr'
                    offered_qos_profiles=empty_qos_str # ここは空文字列のまま
                )

                # TopicMetadata オブジェクトを create_topic に渡す
                writer.create_topic(topic_metadata)

            while reader.has_next():
                (topic_name, data, timestamp) = reader.read_next()
                if topic_name in topics_to_filter:
                    writer.write(topic_name, data, timestamp)

            QtWidgets.QMessageBox.information(self, "Message", "Finish Convert!!")

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Self-implementationでの変換に失敗しました:\n{e}")
        finally:
            if 'reader' in locals() and reader:
                try: reader.close()
                except: pass
            if 'writer' in locals() and writer:
                try: writer.close()
                except: pass


    def on_all_check(self):
        for checkbox in self._checkboxs:
            checkbox.setChecked(True)

    def on_all_uncheck(self):
        for checkbox in self._checkboxs:
            checkbox.setChecked(False)

    def create_menu_bar(self):
        """create menu bar
        """
        
        load_action = self._create_menu('&Load rosbag', self.set_rosbag_topic, 'Ctrl+O', 'Load rosbag file')
        save_config_action = self._create_menu("&Save config", self.save_config, 'Ctrl+S', 'Save checkbox config')
        load_config_action = self._create_menu("&Load config", self.load_config, 'Ctrl+A', 'Load checkbox config')
        convert_action = self._create_menu('&Convert', self.on_convert, None, 'Convert rosbag')
        exit_action = self._create_menu('&Exit', self.quit, 'Ctrl+Q', 'Exit application')
        menubar = self.menuBar()

        file_menu = menubar.addMenu('&File')
        file_menu.addAction(load_action)
        file_menu.addAction(convert_action)
        file_menu.addAction(exit_action)

        config_menu = menubar.addMenu('&Config')
        config_menu.addAction(save_config_action)
        config_menu.addAction(load_config_action)

    def set_rosbag_topic(self):
        """load rosbag and set rosbag topic information on window
        """
        prev_check_topic = []
        for (checkbox, select) in zip(self._checkboxs, self._topic_list):
            if checkbox.isChecked():
                prev_check_topic.append(select)

        file_paths = FileWindow.get_file_path(isApp=True,caption="Select bag file",filefilter="*.bag *.db3 *.mcap")
        if file_paths[0] == "":
            return

        selected_bag_path = file_paths[0]
        
        if not (selected_bag_path.endswith('.bag') or selected_bag_path.endswith('.db3') or selected_bag_path.endswith('.mcap')):
            QtWidgets.QMessageBox.critical(self, "Error", "Selected file is not a valid ROS 2 bag file (.bag, .db3, or .mcap).")
            return
            
        self._files = [selected_bag_path]
        
        topic_list = self._get_topicList(self._files[0])
        
        if len(topic_list) < 1:
            print("Please select a valid bag file or a bag file containing topics.")
            QtWidgets.QMessageBox.critical(self, "Error", "Please select a valid bag file or a bag file containing topics.")
            return

        self._topic_list = topic_list
        self._checkboxs.clear()
        scroll_area = QScrollArea()
        inner = QWidget()
        box_layout = QVBoxLayout()
        
        msg_label = QLabel(self.msg)
        file_path_label = QLabel(f'rosbag path:{self._files[0]}')
        file_path_label.setToolTip(self._files[0])

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.all_check_btn)
        btn_layout.addWidget(self.all_uncheck_btn)
        btn_layout.addWidget(self.convert_btn)
        
        grid_layout = QGridLayout()
        layoutIndex=0
        for select in self._topic_list:
            checkbox=QCheckBox(select)
            if select in prev_check_topic:
                checkbox.setChecked(True)
            grid_layout.addWidget(checkbox,layoutIndex,0)
            layoutIndex=layoutIndex+1
            self._checkboxs.append(checkbox)

        box_layout.addWidget(msg_label)
        box_layout.addWidget(file_path_label)
        box_layout.addLayout(btn_layout)
        box_layout.addLayout(grid_layout)

        inner.setLayout(box_layout)
        scroll_area.setWidget(inner)
        self.setCentralWidget(scroll_area)

    def save_config(self):
        """save checkbox config
        """
        output_path = FileWindow.save_file(isApp=True, caption='Save config file', filefilter='')
        if output_path[0] == "":
            return

        with open(output_path[0], 'w') as f:
            for (checkbox, select) in zip(self._checkboxs, self._topic_list):
                if checkbox.isChecked():
                    f.write(f'{select}\n')

    def load_config(self):
        """load checkbox config
        """
        file_paths = FileWindow.get_file_path(isApp=True,caption='Select config file',filefilter='')
        if file_paths[0] == "":
            return

        checked_list = []
        with open(file_paths[0], 'r') as f:
            checked_list = f.readlines()

        if len(checked_list) == 0:
            return

        checked_list = [i.rstrip('\n') for i in checked_list]
        for (checkbox, select) in zip(self._checkboxs, self._topic_list):
            if  select in checked_list:
                checkbox.setChecked(True)
        

    def _get_topicList(self, bag_file_path):
        reader = SequentialReader()
        input_uri_for_reader = None
        input_storage_id = None
        
        if os.path.isdir(bag_file_path):
            db3_files = glob.glob(os.path.join(bag_file_path, '*.db3'))
            mcap_files = glob.glob(os.path.join(bag_file_path, '*.mcap'))
            if db3_files:
                input_uri_for_reader = bag_file_path
                input_storage_id = 'sqlite3'
            elif mcap_files:
                input_uri_for_reader = bag_file_path
                input_storage_id = 'mcap'
            else:
                QtWidgets.QMessageBox.critical(self, "Error", f"No .db3 or .mcap files found in bag directory: {bag_file_path}")
                return []
        else:
            input_uri_for_reader = os.path.dirname(bag_file_path)
            if not input_uri_for_reader: input_uri_for_reader = "."
            
            if bag_file_path.endswith('.db3'):
                input_storage_id = 'sqlite3'
            elif bag_file_path.endswith('.mcap'):
                input_storage_id = 'mcap'
            elif bag_file_path.endswith('.bag'):
                input_storage_id = 'sqlite3'
            else:
                QtWidgets.QMessageBox.critical(self, "Error", f"Unsupported bag file extension: {bag_file_path}. Please select a .db3, .mcap, or .bag file.")
                return []

        try:
            storage_options = StorageOptions(uri=input_uri_for_reader, storage_id=input_storage_id)
            converter_options = ConverterOptions()

            reader.open(storage_options, converter_options)
            topic_types = reader.get_all_topics_and_types()
            
            topics = []
            for topic_type_tuple in topic_types:
                topics.append(topic_type_tuple.name)
            
            del reader

            return sorted(list(set(topics)))

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to read rosbag2 file '{bag_file_path}': {e}")
            return []

    def _create_menu(self, name, connect, short_cut=None, tooltip=None):
        action = QAction(name, self)
        action.triggered.connect(connect)

        if short_cut is not None:
            action.setShortcut(short_cut) 

        if tooltip is not None:
            action.setStatusTip(tooltip)

        return action

    def quit(self):
        self._app.quit()


def main():
    app = QtWidgets.QApplication(sys.argv)
    main_window=MainWindow(app=app,msg="Select topics which you want to keep", title="rosbag filter")
    app.exec_()


if __name__ == '__main__':
    print("rosbag_filter_gui start!!")
    main()
    print("rosbag_filter_gui end!!")