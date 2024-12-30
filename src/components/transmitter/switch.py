from src.components import Component
from src.utils.network import (
    ZMQKeypointSubscriber,
    ZMQStringPublisher,
    create_response_socket,
)
from src.utils.timer import FrequencyTimer, SwitchStationTimer
from src.constants import *

import numpy as np
import sys
import os
# import keyboard
################################################################
from pynput import keyboard

# 用来存储按键的状态
key_states = {}

# 键盘按下事件的回调
def on_press(key):
    try:
        # 将按下的键记录为 True
        key_states[key.char] = True
    except AttributeError:
        # 如果是特殊键，记录为 True
        key_states[str(key)] = True

# 键盘松开事件的回调
def on_release(key):
    try:
        # 将松开的键记录为 False
        key_states[key.char] = False
    except AttributeError:
        # 如果是特殊键，记录为 False
        key_states[str(key)] = False

    # 退出监听（按下 Esc 键）
    if key == keyboard.Key.esc:
        return False

# 启动键盘监听器
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()
# 检查某个键是否被按下
def is_pressed(key):
    return key_states.get(key, False)
################################################################

class Switch(Component):
    def __init__(
        self,
        host,
        transformed_right_keypoints_port,
        switch_state_port,
        record_varify_port,
    ):
        super().__init__()
        self.notify_component_start("switch")

        # listen for right hand info to set state infomation
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_right_keypoints_port,
            topic="transformed_hand_frame",
        )
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_right_keypoints_port,
            topic="transformed_hand_coords",
        )

        self.state_publisher = ZMQStringPublisher(host, switch_state_port)

        self.record_varify_socket = create_response_socket(host, record_varify_port)

        # self.state = STATUS_STOP
        self.state = STATUS_READY

        self.timer = FrequencyTimer(VR_FREQ)

        self.station_timer = SwitchStationTimer(2, 0.05)

        self.operation_station_timer = SwitchStationTimer(2, 0.05)

    def switch_state(self):
        if self.state == STATUS_STOP:
            self.state = STATUS_READY
        elif self.state == STATUS_READY:
            self.state = STATUS_RUNNING
        else:
            self.state = STATUS_STOP

    def stream(self):
        pre_result = False
        operation_pre_result = False

        while True:
            try:
                self.timer.start_loop()

                if self.state == STATUS_STOP:
                    # wait for storing task completed
                    self.record_varify_socket.recv()
                    self.record_varify_socket.send(b"")
                    self.switch_state()

                # # get right hand infomation
                # transformed_hand_coords = (
                #     self._transformed_hand_keypoint_subscriber.recv_keypoints()
                # )
                # pinky_thumb_distance = np.linalg.norm(
                #     transformed_hand_coords[OCULUS_JOINTS["pinky"][-1]]
                #     - transformed_hand_coords[OCULUS_JOINTS["thumb"][-1]]
                # )
                # middle_thumb_distance = np.linalg.norm(
                #     transformed_hand_coords[OCULUS_JOINTS["middle"][-1]]
                #     - transformed_hand_coords[OCULUS_JOINTS["thumb"][-1]]
                # )

                # result = self.station_timer.trigger(pinky_thumb_distance)
                # if pre_result == False and result == True:
                #     self.switch_state()
                # pre_result = result

                # TODO: close the operation
                # operation_result = self.operation_station_timer.trigger(
                #     middle_thumb_distance
                # )
                # if operation_pre_result == False and operation_result == True:
                #     pass
                # operation_pre_result = operation_result

                # if keyboard.is_pressed("a") and self.state == STATUS_READY:
                #     self.state = STATUS_RUNNING
                # if keyboard.is_pressed("f") and self.state == STATUS_RUNNING:
                #     self.state = STATUS_STOP
                if is_pressed("a") and self.state == STATUS_READY:
                    self.state = STATUS_RUNNING
                if is_pressed("f") and self.state == STATUS_RUNNING:
                    self.state = STATUS_STOP
                # if keyboard.is_pressed("r"):
                #     self.state = STATUS_RESET

                self.state_publisher.pub_string(self.state, "state")

                self.timer.end_loop()

            except KeyboardInterrupt:
                break

        self._transformed_arm_keypoint_subscriber.stop()
        self._transformed_hand_keypoint_subscriber.stop()
        self.state_publisher.stop()
        self.record_varify_socket.close()
        print("Stopping the switch")
