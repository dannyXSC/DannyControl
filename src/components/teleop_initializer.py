from multiprocessing import Process

from src.components.initializers import ProcessInstantiator, _start_component
import sys


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

        if "transmitters" in configs.robot:
            self._init_transmitter()

        if configs.operate:
            self._init_operator()

    # Function to start the detector component
    def _init_detector(self):
        self.processes.append(
            Process(target=_start_component, args=(self.configs.robot.detector,))
        )

    def _init_transmitter(self):
        for transmitter_config in self.configs.robot.transmitters:
            self.processes.append(
                Process(target=_start_component, args=(transmitter_config,))
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
