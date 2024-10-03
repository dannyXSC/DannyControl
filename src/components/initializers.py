import os
import hydra
from abc import ABC
from multiprocessing import Process

import sys


class ProcessInstantiator(ABC):
    def __init__(self, configs):
        self.configs = configs
        self.processes = []

    def _start_component(self, configs):
        raise NotImplementedError("Function not implemented!")

    def get_processes(self):
        return self.processes


# Function to start the components
def _start_component(configs):
    component = hydra.utils.instantiate(configs)
    component.stream()
