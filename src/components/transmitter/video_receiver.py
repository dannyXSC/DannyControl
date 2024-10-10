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
import datetime
import json


class VideoStreamer(object):
    def __init__(self, host, cam_port, height, width, cam_name, serial_num):
        self._init_socket(host, cam_port)
        self.height = height
        self.width = width
        self.transforms = transforms.Compose(
            [transforms.ToPILImage(), transforms.ToTensor()]
        )
        self.cam_name = cam_name
        self.serial_num = serial_num

    def get_cam_name(self):
        return self.cam_name

    def get_serial_num(self):
        return self.serial_num

    def _init_socket(self, host, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect("tcp://{}:{}".format(host, port))
        self.socket.setsockopt(zmq.SUBSCRIBE, b"rgb_image")

    def get_image_tensor(self):
        raw_data = self.socket.recv()
        data = raw_data.lstrip(b"rgb_image ")
        data = pickle.loads(data)
<<<<<<< HEAD

        encoded_rgb = np.frombuffer(base64.b64decode(data["rgb_image"]), np.uint8)
        decoded_rgb = cv2.imdecode(encoded_rgb, cv2.IMREAD_COLOR)

        # timestamp = data["timestamp"]
=======
        encoded_rgb = np.frombuffer(base64.b64decode(data['rgb_image']), np.uint8)
        decoded_rgb = cv2.imdecode(encoded_rgb, cv2.IMREAD_COLOR).reshape(3,self.height,self.width)
        timestamp = data['timestamp']
        # file_path = f'D:\project\project1\code\data\\time_{timestamp}.jpg'
        # with open(file_path, 'wb') as f:
        #     f.write(encoded_rgb)
        # print(f"Image saved to: {file_path}")
        # image = Image.open(file_path)
        # image = np.array(image)
        # image_tensor = self.transforms(image)
        # print(image_tensor.shape)
        # return encoded_rgb, timestamp
        # return encoded_rgb
>>>>>>> data_process
        return decoded_rgb, self.get_cam_name()

    def process_frames(self):

        image_tensor, timestamp = self._get_image_tensor()
        image_data = {
            "encoded_rgb": image_tensor.tolist(),
            # "encoded_depth": encoded_depth.tolist(),
            "timestamp": timestamp,
        }
        json_file_path = f"D:\project\project1\code\data\\visual\\time_{timestamp}.json"
        with open(json_file_path, "w") as json_file:
            json.dump(image_data, json_file)
        print(f"Image data saved to: {json_file_path}")
        print("Timestamp:", timestamp)
        print("Image shape:", image_tensor.shape)


class VideoReceiver(object):
    def __init__(self, configs):
        # Loading the network configurations
        self.host_address = configs.host_address
        self.port_offset = configs.cam_port_offset
        self.num_cams = len(configs.camera_info)
        self.image_height = configs.cam_configs.height
        self.image_width = configs.cam_configs.width
        # self.robot_cam_serial_numbers = configs.robot_cam_serial_numbers
        self.camera_pairs = configs.camera_info

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
        for cam_idx, pair in enumerate(self.camera_pairs):
            cam_name = pair.name
            cam_serial_num = pair.serial_number
            self.cam_streamers.append(
                VideoStreamer(
                    host=self.host_address,
                    cam_port=self.port_offset + cam_idx,
                    height=self.image_height,
                    width=self.image_width,
                    cam_name=cam_name,
                    serial_num=cam_serial_num,
                )
            )

    def get_cam_streamer(self, id):
        return self.cam_streamers[id - 1]
