import hydra
from src.components import RealsenseCameras


@hydra.main(version_base="1.2", config_path="configs", config_name="camera")
def main(configs):
    # camera_pairs = configs.robot_cam_serial_numbers
    # for pair in camera_pairs:
    #     for key, value in pair.items():
    #         print(key)
    #         print(value)

    cameras = RealsenseCameras(configs)
    processes = cameras.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()


if __name__ == "__main__":
    main()
