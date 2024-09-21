import zmq
import hydra
import base64
import numpy as np
import pickle
from torchvision import transforms
import time
import cv2
import os
from PIL import Image

class VideoStreamer(object):
    def __init__(self, host, cam_port, height, width):
        self._init_socket(host, cam_port)
        self.height = height
        self.width = width
        self.transforms = transforms.Compose([
            transforms.ToPILImage(),
            transforms.ToTensor()
        ])

    def _init_socket(self, host, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect("tcp://{}:{}".format(host, port))
        self.socket.setsockopt(zmq.SUBSCRIBE, b"rgb_image")


    @hydra.main(version_base="1.2", config_path="configs", config_name="camera")
    def _get_image_tensor(self):
        raw_data = self.socket.recv()
        data = raw_data.lstrip(b"rgb_image ")
        data = pickle.loads(data)
        encoded_data = np.frombuffer(base64.b64decode(data['rgb_image']), np.uint8)
        timestamp = data['timestamp']
        # image = cv2.imencode('.jpg', encoded_data)[1].tostring()
        file_path = f'D:\project\project1\code\data\\time_{timestamp}.jpg'
        with open(file_path, 'wb') as f:
            f.write(encoded_data)
        print(f"Image saved to: {file_path}")

        image = Image.open(file_path)
        image = np.array(image)
        image_tensor = self.transforms(image) 
        timestamp = time.time()
        
        return image_tensor, timestamp
    
    def process_frames(self):
        while True:
            image_tensor, timestamp = self._get_image_tensor()
            print("Timestamp:", timestamp)
            print("Image shape:", image_tensor.shape)

class ReceiverApplication(object):
    def __init__(self, configs):
        # Loading the network configurations
        self.host_address = configs.host_address
        self.keypoint_port = configs.keypoint_port
        self.port_offset = configs.cam_port_offset
        self.num_cams = len(configs.robot_cam_serial_numbers)
        self.image_height = configs.cam_configs.height
        self.image_width = configs.cam_configs.width

        # Initializing the streamers        
        self._init_cam_streamers()
        self._init_graph_streamer()

        # Initializing frequency checkers
        self._init_frequency_checkers()
        
    def _init_graph_streamer(self):
        # TODO
        pass

    def _init_frequency_checkers(self):
        # TODO - Raw keypoint frequency
        # TODO - Transformed keypoint frequency
        pass

    def _init_cam_streamers(self):
        self.cam_streamers = []
        for idx in range(self.num_cams):
            self.cam_streamers.append(
                VideoStreamer(
                    host = self.host_address,
                    cam_port = self.port_offset + idx, 
                    height = self.image_height,
                    width = self.image_width
                )
            )

    def get_cam_streamer(self, id):
        return self.cam_streamers[id - 1]