import hydra
from src.components.record_initializer import Recorder


@hydra.main(version_base="1.2", config_path="configs", config_name="recorder")
def main(configs):
    recorder = Recorder(configs)
    processes = recorder.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()


if __name__ == "__main__":
    main()
