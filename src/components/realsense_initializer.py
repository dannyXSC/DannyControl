import os
import hydra
from abc import ABC
from multiprocessing import Process

from src.components.sensors import *
from src.components.initializers import ProcessInstantiator


def _start_component(
    configs,
    cam_idx,
    cam_name,
    cam_serial_num,
    stream_rescale_factor,
):
    component = RealsenseCamera(
        stream_configs=dict(
            host=configs.host_address,
            port=configs.cam_port_offset + cam_idx,
        ),
        cam_name=cam_name,
        cam_serial_num=cam_serial_num,
        cam_id=cam_idx + 1,
        cam_configs=configs.cam_configs,
        stream_oculus=True if configs.oculus_cam == cam_idx else False,
        stream_rescale_factor=stream_rescale_factor,
    )
    component.stream()


class RealsenseCameras(ProcessInstantiator):
    """
    Returns all the camera processes. Start the list of processes to start
    the camera stream.
    """

    def __init__(self, configs):
        super().__init__(configs)
        # Creating all the camera processes
        self._init_camera_processes()

    def _init_camera_processes(self):
        camera_pairs = self.configs.camera_info
        for cam_idx, pair in enumerate(camera_pairs):
            cam_name = pair.name
            cam_serial_num = pair.serial_number
            cam_stream_rescale_factor = pair.stream_rescale_factor
            self.processes.append(
                Process(
                    target=_start_component,
                    args=(
                        self.configs,
                        cam_idx,
                        cam_name,
                        cam_serial_num,
                        cam_stream_rescale_factor,
                    ),
                )
            )
