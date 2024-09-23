import os
import hydra
from abc import ABC
from multiprocessing import Process

from .sensors import *


class ProcessInstantiator(ABC):
    def __init__(self, configs):
        self.configs = configs
        self.processes = []

    def _start_component(self, configs):
        raise NotImplementedError("Function not implemented!")

    def get_processes(self):
        return self.processes


class RealsenseCameras(ProcessInstantiator):
    """
    Returns all the camera processes. Start the list of processes to start
    the camera stream.
    """

    def __init__(self, configs):
        super().__init__(configs)
        # Creating all the camera processes
        self._init_camera_processes()

    def _start_component(self, cam_idx):
        component = RealsenseCamera(
            stream_configs=dict(
                host=self.configs.host_address,
                port=self.configs.cam_port_offset + cam_idx,
            ),
            cam_serial_num=self.configs.robot_cam_serial_numbers[cam_idx],
            cam_id=cam_idx + 1,
            cam_configs=self.configs.cam_configs,
            stream_oculus=True if self.configs.oculus_cam == cam_idx else False,
        )
        component.stream()

    def _init_camera_processes(self):
        for cam_idx in range(len(self.configs.robot_cam_serial_numbers)):
            self.processes.append(
                Process(target=self._start_component, args=(cam_idx,))
            )


# Function to start the components
def _start_component(configs):
    component = hydra.utils.instantiate(configs)
    component.stream()


class TeleOperator(ProcessInstantiator):
    """
    Returns all the teleoperation processes. Start the list of processes
    to run the teleop.
    """

    def __init__(self, configs):
        super().__init__(configs)

        # For Simulation environment start the environment as well
        if configs.sim_env:
            self._init_sim_environment()
        # Start the Hand Detector
        if "detector" in configs.robot:
            self._init_detector()
        # Start the keypoint transform
        if "transforms" in configs.robot:
            self._init_keypoint_transform()

        # if configs.operate:
        #     self._init_operator()

    # Function to start the detector component
    def _init_detector(self):
        self.processes.append(
            Process(target=_start_component, args=(self.configs.robot.detector,))
        )

    # Function to start the sim environment
    def _init_sim_environment(self):
        for env_config in self.configs.robot.environment:
            self.processes.append(Process(target=_start_component, args=(env_config,)))

    # Function to start the keypoint transform
    def _init_keypoint_transform(self):
        for transform_config in self.configs.robot.transforms:
            self.processes.append(
                Process(target=_start_component, args=(transform_config,))
            )

    # Function to start the operator
    def _init_operator(self):
        for operator_config in self.configs.robot.operators:

            self.processes.append(
                Process(target=_start_component, args=(operator_config,))
            )
