from src.components.sensors.realsense import RealsenseHamal
from src.components.transmitter.video_receiver import VideoReceiver
import hydra
from src.utils.timer import LogTimer
import time

hydra.initialize(config_path="./configs", version_base="1.2")
configs = hydra.compose("server")
video_receiver = VideoReceiver(configs)

logger = LogTimer(1)

start_time = time.time()
cnt = 0
while True:
    for id in range(configs.num_cams):
        rgb_data, cam_name = video_receiver.get_cam_streamer(id).get_image_tensor()
        cnt += 1
        duration = time.time() - start_time
        logger.trigger(f"frequency: {cnt/(duration)}")
