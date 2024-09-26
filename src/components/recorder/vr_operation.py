from src.utils.network import (
    ZMQPayloadSubscriber,
    ZMQStringSubscriber,
    ZMQKeypointSubscriber,
)
from src.utils.timer import FrequencyTimer
from src.constants import *
import h5py
import numpy as np
from abc import ABC, abstractmethod
import os
from src.components import Component


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
    def save(self, file_name, metadata=None):
        pass


class VROpH5pyDumper(H5pyDumper):
    def __init__(self) -> None:
        super().__init__()

    def save(self, file_name, metadata=None):
        camera_key = "cam_data"
        with h5py.File(file_name, "w") as file:
            camera = file.create_group(camera_key)
            
            for key in self.data_dict.keys():
                if key == camera_key:
                    for cam_name in self.data_dict[camera_key]:
                        # TODO image shape
                        # self.data_dict[camera_key][cam_name] = np.array(
                        #     self.data_dict[camera_key][cam_name], dtype=np.uint16
                        # )
                        camera.create_dataset(
                            cam_name,
                            data=self.data_dict[camera_key][cam_name],
                            chunks=(1, 720, 1280, 3),
                            compression="gzip",
                            compression_opts=6,
                        )
                elif key == "step":
                    self.data_dict[key] = np.array(self.data_dict[key], dtype=np.int64)
                    file.create_dataset(
                        key,
                        data=self.data_dict[key],
                        compression="gzip",
                        compression_opts=6,
                    )
                else:
                    self.data_dict[key] = np.array(
                        self.data_dict[key], dtype=np.float32
                    )
                    file.create_dataset(
                        key,
                        data=self.data_dict[key],
                        compression="gzip",
                        compression_opts=6,
                    )


class VROperationRecorder(Component):

    def __init__(self, host, listen_port, switch_state_port, save_path) -> None:
        self.payload_subscriber = ZMQKeypointSubscriber(
            host, listen_port, XARM_NOTIFIER_TOPIC
        )
        self.state_subscriber = ZMQStringSubscriber(host, switch_state_port, "state")

        self.timer = FrequencyTimer(TRANS_FREQ)
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
                
                if state == STATUS_STOP:
                    if not self.h5py_dumper.empty():
                        target_path = os.path.join(
                            f"{self.save_path}", f"episode_{cnt}.hdf5"
                        )
                        self.h5py_dumper.save(target_path)
                        self.h5py_dumper.init()

                        cnt += 1
                    self.timer.end_loop()
                    continue

                payload = self._get_payload()
                print(payload['cam_data']['robot_camera'])
                print(type(payload['cam_data']['robot_camera']))
                print(payload['cam_data']['robot_camera'].shape)


                # 调整payload的格式

                # 用h5py对 payload 进行保存
                self.h5py_dumper.add(payload)

                self.timer.end_loop()
            except KeyboardInterrupt:
                break

        self.payload_subscriber.stop()

        print("Stopping the recorder.")
