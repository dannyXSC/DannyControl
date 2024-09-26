import hydra
import sys
import os

from torchgen.gen_lazy_tensor import default_args

sys.path.append("../../..")
sys.path.append("../..")
sys.path.append("../")
from docutils.utils.math.latex2mathml import operators
from pyarrow import timestamp

from video_receiver import VideoReceiver
from xarm_info_transmitter import XarmInfoReceiver
from src.components.operators.xarm import XarmOperator
from src.constants import XARM_NOTIFIER_TOPIC
from src.utils.network import ZMQKeypointSubscriber
from multiprocessing import Process
import json, time
import argparse



def arg_parse():
    parse = argparse.ArgumentParser(description='receiver')
    parse.add_argument('--task_name', default='demo_task', type=str, help='task name')
    parse.add_argument('--index', default=0, type=int, help='the index of the demonstration')
    parse.add_argument('--path', default='D:\project\project1\code\data', type=str, help='path for data')

    args = parse.parse_args()
    return args


def main(args):
    # Initializing the monitor class
    hydra.initialize(config_path='../../../configs', version_base='1.2')
    configs = hydra.compose('server')
    video_receiver = VideoReceiver(configs)

    xarm_ip = '10.177.63.209'
    xarm_operator = XarmOperator(configs.host_address, configs.transformed_position_keypoint_port, xarm_ip,
                                 [300, 300, 500],
                                 0.5)
    xarm_subscriber = ZMQKeypointSubscriber(configs.host_address, configs.xarm_info_transmitter_port,
                                            XARM_NOTIFIER_TOPIC)

    task_name = args.task_name
    index = args.index
    data_path = args.path
    file_name = task_name + '_episode_' + str(index) + '.json'
    file = os.path.join(data_path, file_name)

    data = []
    step = 0

    with open(file, 'w') as f:
        while True:
            try:
                gripper_state_from_quest = xarm_operator.get_gripper_state()
                transformed_hand_frames = xarm_operator.transformed_arm_keypoint_subscriber.recv_keypoints()
                transformed_hand_coords = xarm_operator.transformed_hand_keypoint_subscriber.recv_keypoints()
                keypoints = xarm_subscriber.recv_keypoints()
                # joint_angle = keypoints['joint_angle']
                end_position = keypoints['end_position']
                # gripper_state_from_xarm = keypoints['gripper_state']
                cam_data = []
                timestamp = time.time()

                for id in range(configs.num_cams):
                    # rgb_data, timestamp = video_receiver.get_cam_streamer(id).get_image_tensor()
                    rgb_data, cam_name = video_receiver.get_cam_streamer(id).get_image_tensor()
                    # cam_data.append({
                    #     "serial_num": video_receiver.get_cam_streamer(id).get_serial_num(),
                    #     "rgb_image": rgb_data.tolist()
                    # })\
                    cam_data.append({
                        cam_name: rgb_data.tolist(),
                    })

                entry = {
                    "step": step,
                    "timestamp": timestamp,
                    "gripper_state": gripper_state_from_quest,
                    "transformed_hand_frames": transformed_hand_frames.tolist(),
                    "transformed_hand_coords": transformed_hand_coords.tolist(),
                    "end_position": end_position.tolist(),
                    "cam_data": cam_data
                }

                data.append(entry)

                # print("Gripper state", gripper_state_from_quest)
                # print("Xarm: transformed_hand_frames: ", transformed_hand_frames)
                # print("Xarm: transformed_hand_coords: ", transformed_hand_coords)
                # print("keypoints: ", keypoints)
                # for cam_entry in cam_data:
                #     print("Cam ID:", cam_entry["serial_num"])
                #     print("Timestamp:", cam_entry["timestamp"])
                #     print("Image shape:", cam_entry["image_shape"])

                json.dump(data, f, indent=4)
                data = []
                step += 1

            except Exception as e:
                print(e)
                break

    # with open(file, 'r') as f:
    #     while True:
    #         try:
    #             gripper_state = xarm_operator.get_gripper_state()
    #             transformed_hand_frames = xarm_operator.transformed_arm_keypoint_subscriber.recv_keypoints()
    #             transformed_hand_coords = xarm_operator.transformed_hand_keypoint_subscriber.recv_keypoints()
    #             print("Gripper state", gripper_state)
    #             print("Xarm: transformed_hand_frames: ", transformed_hand_frames)
    #             print("Xarm: transformed_hand_coords: ", transformed_hand_coords)
    #             keypoints = xarm_subscriber.recv_keypoints()
    #             print("keypoints: ", keypoints)
    #             for id in range(configs.num_cams):
    #                 rgb_data, timestamp = video_receiver.get_cam_streamer(id).get_image_tensor()
    #                 print("Timestamp:", timestamp)
    #                 print("Image shape:", rgb_data.shape)
    #         except Exception as e:
    #             print(e)
    #             break


if __name__ == '__main__':
    args = arg_parse()
    main(args)
