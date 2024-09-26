from src.components import Component
from src.utils.timer import FrequencyTimer
from src.constants import *
from src.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from src.components.robot.xarm import Xarm
from src.data.xarm import XarmInfo
from video_receiver import VideoReceiver
from src.components.operators.xarm import XarmOperator
import hydra
import time,json


class XarmInfoNotifier(Component):
    def __init__(self, host, xarm_info_transmitter_port, transformed_position_keypoint_port, xarm_ip, scale_vector, comp_ratio):
        self.notify_component_start("xarm notifier")

        hydra.initialize(config_path='../../../configs', version_base='1.2')
        self.configs = hydra.compose('server')
        self.video_receiver = VideoReceiver(self.configs)
        self.xarm_operator = XarmOperator(host, transformed_position_keypoint_port, xarm_ip, scale_vector, comp_ratio)
        self.xarm_publisher = ZMQKeypointPublisher(host, xarm_info_transmitter_port)
        self.timer = FrequencyTimer(TRANS_FREQ)

        self.robot = Xarm(xarm_ip)

    def stream(self):
        step = 0
        while True:
            if self.robot.if_shutdown():
                continue

            try:
                self.timer.start_loop()

                payload = {}

                gripper_state_from_quest = self.xarm_operator.get_gripper_state()
                transformed_hand_frames = self.xarm_operator.transformed_arm_keypoint_subscriber.recv_keypoints()
                transformed_hand_coords = self.xarm_operator.transformed_hand_keypoint_subscriber.recv_keypoints()
                end_position = self.robot.get_cartesian_position()
                cam_data = []
                timestamp = time.time()
                for id in range(self.configs.num_cams):
                    rgb_data, cam_name = self.video_receiver.get_cam_streamer(id).get_image_tensor()
                    cam_data.append({
                        cam_name: rgb_data,
                    })


                payload['step'] = step
                payload['timestamp'] = timestamp
                payload['gripper_state'] = gripper_state_from_quest
                payload['transformed_hand_frames'] = transformed_hand_frames
                payload['transformed_hand_coords'] = transformed_hand_coords
                payload['end_position'] = end_position
                payload['cam_data'] = cam_data

                json_payload = json.dumps(payload)


                # xarm_info = (
                #     XarmInfo()
                #     .set_joint_angle(self.robot.get_joint_position())
                #     .set_end_position(self.robot.get_cartesian_position())
                # )
                #
                # payload = xarm_info.to_json()

                self.xarm_publisher.pub_keypoints(
                    json_payload, topic_name=XARM_NOTIFIER_TOPIC
                )

                # End the timer
                self.timer.end_loop()
                step += 1

            except Exception as e:
                print(e)
                break

        self.xarm_publisher.stop()
        self.robot.stop()
        print("Stopping the xarm notifier process.")


class XarmInfoReceiver(object):
    def __init__(self, host, port):
        self.xarm_subscriber = ZMQKeypointSubscriber(host, port, XARM_NOTIFIER_TOPIC)

    def get_info(self):
        payload = self.xarm_subscriber.recv_keypoints()
        data = XarmInfo().from_json(payload)
        return data
