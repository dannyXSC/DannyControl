import hydra
from video_receiver import ReceiverApplication


# Initializing the monitor class
hydra.initialize(config_path = '../../../configs', version_base = '1.2')
configs = hydra.compose('server')
receiver = ReceiverApplication(configs)

def load_from_camera(id):
    receiver.get_cam_streamer(id).process_frames()
    return 

if __name__ == '__main__':
    for id in range(configs.num_cams):
        load_from_camera(id)
