from src.utils.network import ZMQPayloadSubscriber
from src.utils.timer import FrequencyTimer
from src.constants import *
import h5py
import numpy as np
from abc import ABC, abstractmethod
import os


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
        pass


class VROperationRecorder(object):
    def __init__(self, listen_host, listen_port, save_path) -> None:
        # TODO: 接收一些停止信号
        self.payload_subscriber = VROpPayloadSubscriber(listen_host, listen_port)
        self.timer = FrequencyTimer(RECORD_FREQ)
        self.h5py_dumper = H5pyDumper()
        self.save_path = save_path

    def _get_payload(self):
        payload = self.payload_subscriber.recv_payload()
        return payload

    def stream(self):
        while True:
            try:
                self.timer.start_loop()

                payload = self._get_payload()

                # 调整payload的格式

                # 用h5py对 payload 进行保存
                self.h5py_dumper.add(payload)

                self.timer.end_loop()
            except:
                break

        self.h5py_dumper.save(os.path.join(f"{self.save_path}", f"episode_{0}.hdf5"))

        self.payload_subscriber.stop()

        print("Stopping the recorder.")
