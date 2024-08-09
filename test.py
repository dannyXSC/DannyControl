import hydra
from src.components import TeleOperator
import multiprocessing as mp


@hydra.main(version_base="1.2", config_path="configs", config_name="teleop")
def main(configs):
    teleop = TeleOperator(configs)
    processes = teleop.get_processes()

    processes[1].start()
    processes[1].join()

    # for process in processes:
    #     process.start()

    # for process in processes:
    #     process.join()


if __name__ == "__main__":
    main()
