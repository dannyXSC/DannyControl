from src.components.initializers import ProcessInstantiator, _start_component
from multiprocessing import Process


class Recorder(ProcessInstantiator):
    def __init__(self, configs):
        super().__init__(configs)

        self._init_recorder()

    def _init_recorder(self):
        for recorder_config in self.configs.recorder:
            self.processes.append(
                Process(target=_start_component, args=(recorder_config,))
            )
