import os
import hydra
from abc import ABC
from .recorder.image import RGBImageRecorder, DepthImageRecorder
from .recorder.robot_state import RobotInformationRecord
from .sensors import *
from multiprocessing import Process
from src.constants import *
from src.components.initializers import ProcessInstantiator, _start_component
import re


class Recorder(ProcessInstantiator):
    def __init__(self, configs):
        super().__init__(configs)

        self._init_recorder()

    def _init_recorder(self):
        for recorder_config in self.configs.recorder:
            self.processes.append(
                Process(target=_start_component, args=(recorder_config,))
            )


class Collector(ProcessInstantiator):
    """
    Returns all the recorder processes. Start the list of processes
    to run the record data.
    """

    def __init__(self, configs):
        super().__init__(configs)
        self.demo_num = self._get_next_demo_num()  # Automatically set demo_num
        # read storage path dictname 
        # get the max number of the dictname
        self._storage_path = os.path.join(
            self.configs.storage_path,
            'demonstration_{}'.format(self.demo_num)
        )

        self._create_storage_dir()
        self._init_camera_recorders()
        # Initializing the recorders
        print("Initialising robot recorders")
        self._init_robot_recorders()

    def _get_next_demo_num(self):
        """
        Get the next demonstration number based on existing directories.
        """
        if not os.path.exists(self.configs.storage_path):
            os.mkdir(self.configs.storage_path)
            return 0  # Start from 1 if the storage path doesn't exist

        existing_dirs = os.listdir(self.configs.storage_path)
        demo_nums = []

        # Extract numbers from directories matching 'demonstration_{num}'
        for dir_name in existing_dirs:
            match = re.match(r'demonstration_(\d+)', dir_name)
            if match:
                demo_nums.append(int(match.group(1)))

        return max(demo_nums, default=0) + 1  # Return the next number

    def _create_storage_dir(self):
        if os.path.exists(self._storage_path):
            return
        else:
            os.makedirs(self._storage_path)

    # Function to start the components
    def _start_component(self, component):
        component.stream()

    # Record the rgb components
    def _start_rgb_component(self, cam_idx=0):
        # This part has been isolated and made different for the sim and real robot
        # If using simulation and real robot on the same network, only one of them will stream into the VR. Close the real robot realsense camera stream before launching simulation.
        print("RGB function")
        component = RGBImageRecorder(
            host=self.configs.host_address,
            image_stream_port=self.configs.cam_port_offset + cam_idx,
            storage_path=self._storage_path,
            filename='cam_{}_rgb_video'.format(cam_idx)
        )
        component.stream()

    # Record the depth components
    def _start_depth_component(self, cam_idx):
        component = DepthImageRecorder(
            host=self.configs.host_address,
            image_stream_port=self.configs.cam_port_offset + cam_idx + DEPTH_PORT_OFFSET,
            storage_path=self._storage_path,
            filename='cam_{}_depth'.format(cam_idx)
        )
        component.stream()

    # Function to start the camera recorders
    def _init_camera_recorders(self):
        print("Camera recorder starting")
        for cam_idx in range(len(self.configs.camera_info)):
            # print(cam_idx)
            self.processes.append(Process(
                target=self._start_rgb_component,
                args=(cam_idx,)
            ))

            self.processes.append(Process(
                target=self._start_depth_component,
                args=(cam_idx,)
            ))

    # Function to start the robot recorders
    def _start_robot_component(
            self,
            robot_configs,
            recorder_function_key):
        component = RobotInformationRecord(
            robot_configs=robot_configs,
            recorder_function_key=recorder_function_key,
            storage_path=self._storage_path
        )

        component.stream()

    # Function to start the robot recorders
    def _init_robot_recorders(self):
        # Instantiating the robot classes
        for idx, robot_controller_configs in enumerate(self.configs.robot.controllers):
            for key in self.configs.robot.recorded_data[idx]:
                self.processes.append(Process(
                    target=self._start_robot_component,
                    args=(robot_controller_configs, key,)
                ))
