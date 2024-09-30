import os
import hydra
from abc import ABC
from multiprocessing import Process

from src.components.sensors import *
from src.components.initializers import ProcessInstantiator


class RealsenseCameras(ProcessInstantiator):
    """
    Returns all the camera processes. Start the list of processes to start
    the camera stream.
    """

    def __init__(self, configs):
        super().__init__(configs)
        # Creating all the camera processes
        self._init_camera_processes()

    def _start_component(
        self,
        cam_idx,
        cam_name,
        cam_serial_num,
        stream_rescale_factor,
    ):
        component = RealsenseCamera(
            stream_configs=dict(
                host=self.configs.host_address,
                port=self.configs.cam_port_offset + cam_idx,
            ),
            cam_name=cam_name,
            cam_serial_num=cam_serial_num,
            cam_id=cam_idx + 1,
            cam_configs=self.configs.cam_configs,
            stream_oculus=True if self.configs.oculus_cam == cam_idx else False,
            stream_rescale_factor=stream_rescale_factor,
        )
        component.stream()

    def _init_camera_processes(self):
        camera_pairs = self.configs.camera_info
        for cam_idx, pair in enumerate(camera_pairs):
            cam_name = pair.name
            cam_serial_num = pair.serial_number
            cam_stream_rescale_factor = pair.stream_rescale_factor
            self.processes.append(
                Process(
                    target=self._start_component,
                    args=(cam_idx, cam_name, cam_serial_num, cam_stream_rescale_factor),
                )
            )
