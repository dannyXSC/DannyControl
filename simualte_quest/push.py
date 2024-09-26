#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq

import cv2
import time
import zmq
import pickle
import numpy as np
import base64


class FrequencyTimer(object):
    def __init__(self, frequency_rate):
        self.time_available = 1e9 / frequency_rate

    def start_loop(self):
        self.start_time = time.time_ns()
        print("start time:", self.start_time)

    def end_loop(self):
        wait_time = self.time_available + self.start_time

        while time.time_ns() < wait_time:
            continue


context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect("tcp://10.162.196.203:8087")

print("begin")


with open("./log.txt", "r") as f:
    poses = f.read()
pose_list = poses.split("\n")[:-1]
print(len(pose_list))

timer = FrequencyTimer(10)

for i in range(len(pose_list)):
    timer.start_loop()
    #  Wait for next request from client
    message = socket.send_string(pose_list[i])
    print(i)
    timer.end_loop()

# socket.connect("tcp://10.177.63.88:8110")

# print("begin")

# for i in range(5):
#     #  Wait for next request from client
#     message = socket.send(b"1")
#     print("send")
input()
