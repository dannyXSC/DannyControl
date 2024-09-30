from src.utils.network import (
    ZMQPayloadSubscriber,
    ZMQStringSubscriber,
    ZMQKeypointSubscriber,
    create_request_socket,
)
from src.utils.timer import FrequencyTimer,LogTimer
from src.constants import *
import h5py
import numpy as np
from abc import ABC, abstractmethod
import os
from src.components import Component
from concurrent.futures import ThreadPoolExecutor,as_completed
import time

class VROpPayloadSubscriber(ZMQPayloadSubscriber):
    def __init__(self, host, port):
        super().__init__(host, port)

    def postprocessing(self, payload):
        # TODO: 对图片进行后处理
        return payload


class H5pyDumper(ABC):
    def __init__(self) -> None:
        self.init()

    def init(self):
        self.data_dict = {}

    def empty(self):
        return len(self.data_dict) == 0

    def _dict_append(self, target, source):
        # target source都是字典
        for key, value in source.items():
            if not isinstance(value, dict):
                if key not in target:
                    target[key] = [source[key]]
                else:
                    target[key].append(source[key])
            else:
                if key not in target:
                    target[key] = {}
                self._dict_append(target[key], source[key])

    def add(self, payload):
        # keys in payload must be same all the time
        self._dict_append(self.data_dict, payload)

    @abstractmethod
    def save(self,file_name, metadata=None):
        pass


class VROpH5pyDumper(H5pyDumper):
    def __init__(self,max_workers = 10) -> None:
        super().__init__()
        self.max_workers = max_workers
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.all_task = []

    def _save(data_dict, file_name, metadata=None):
        camera_key = "cam_data"
        with h5py.File(file_name, "w") as file:
            camera = file.create_group(camera_key)

            for key in data_dict.keys():
                if key == camera_key:
                    for cam_name in data_dict[camera_key]:
                        # TODO image shape
                        data_dict[camera_key][cam_name] = np.array(
                            data_dict[camera_key][cam_name], dtype=np.uint16
                        )
                        camera.create_dataset(
                            cam_name,
                            data=data_dict[camera_key][cam_name],
                            chunks=(1, 240, 426, 3),
                            compression="gzip",
                            compression_opts=6,
                        )
                elif key == "step" or key == "timestamp":
                    data_dict[key] = np.array(data_dict[key], dtype=np.int64)
                    file.create_dataset(
                        key,
                        data=data_dict[key],
                        compression="gzip",
                        compression_opts=6,
                    )
                else:
                    data_dict[key] = np.array(
                        data_dict[key], dtype=np.float32
                    )
                    if key=='timestamp':
                        print(data_dict[key])
                    file.create_dataset(
                        key,
                        data=data_dict[key],
                        compression="gzip",
                        compression_opts=6,
                    )
        print(f"H5py saving complete! Path: {file_name}")
    
    def save(self,file_name,metadata=None):
        VROpH5pyDumper._save(self.data_dict,file_name,metadata)
    
    def get_active_tasks(self):
        for task in self.all_task:
            if task.done():
                self.all_task.remove(task)
        return len(self.all_task)
    
    def save_sync(self,file_name,metadata=None):
        while True:
            task_num = self.get_active_tasks()
            # 允许等待 {5} 个任务
            if task_num<self.max_workers + 5:
                break
        self.all_task.append(self.executor.submit(VROpH5pyDumper._save,self.data_dict,file_name,metadata))

    def stop(self):
        for future in as_completed(self.all_task):

            task_num = self.get_active_tasks()
            print("Remain task number: {}".format(task_num))

class VROperationRecorder(Component):
    def __init__(
        self, host, listen_port, switch_state_port, record_varify_port, save_path
    ) -> None:
        self.payload_subscriber = ZMQKeypointSubscriber(
            host, listen_port, XARM_NOTIFIER_TOPIC
        )
        self.state_subscriber = ZMQStringSubscriber(host, switch_state_port, "state")

        self.record_varify_socket = create_request_socket(host, record_varify_port)

        self.timer = FrequencyTimer(RECORD_FREQ)
        self.h5py_dumper = VROpH5pyDumper()
        self.save_path = save_path

    def _get_payload(self):
        payload = self.payload_subscriber.recv_keypoints()
        return payload

    def stream(self):
        self.notify_component_start("VROperationRecorder")
        cnt = 0
        while True:
            try:
                self.timer.start_loop()

                state = self.state_subscriber.recv_payload()

                if state != STATUS_RUNNING:
                    if not self.h5py_dumper.empty():
                        target_path = os.path.join(
                            f"{self.save_path}", f"episode_{cnt}.hdf5"
                        )
                        # self.h5py_dumper.save(target_path)
                        self.h5py_dumper.save_sync(target_path)
                        self.h5py_dumper.init()
                        self.record_varify_socket.send(b"")
                        self.record_varify_socket.recv()

                        cnt += 1
                    self.timer.end_loop()
                    continue
                
                
                payload = self._get_payload()
                # 调整payload的格式

                # 用h5py对 payload 进行保存
                self.h5py_dumper.add(payload)
            

                self.timer.end_loop()
            except KeyboardInterrupt:
                break

        self.payload_subscriber.stop()
        self.record_varify_socket.close()
        
        self.h5py_dumper.stop()

        print("Stopping the recorder.")
