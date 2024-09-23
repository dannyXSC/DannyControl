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

    def end_loop(self):
        wait_time = self.time_available + self.start_time

        while time.time_ns() < wait_time:
            continue


class StationTimer(object):
    def __init__(self, duration, threshold) -> None:
        self.pre_state = None

        self.pre_time = None
        self.duration = duration
        self.threshold = threshold

    def trigger(self, state):
        if not self._validation(state):
            self._set(state)

            return False

        return self._success(state)

    def get_state(self):
        return self.pre_state

    def _set(self, state):
        self.pre_state = state
        self.pre_time = int(time.time())

    def _validation(self, state):
        # if self.pre_state is not None:
        #     print(np.linalg.norm(self.pre_state - state))

        if (
            self.pre_state is not None
            and np.linalg.norm(self.pre_state - state) < self.threshold
        ):
            return True
        return False

    def _success(self, state):
        cur_time = int(time.time())
        # 如果开始，并且相比于第一次记录已经过了duration
        if self.pre_time is not None and cur_time > self.pre_time + self.duration:
            # 记录稳定时候的状态
            self.pre_state = state
            return True
        else:
            return False


class LogTimer(object):

    def __init__(self, duration) -> None:
        self.pre_time = int(time.time())
        self.duration = duration

    def validate(self):
        cur_time = int(time.time())
        return cur_time > self.pre_time + self.duration

    def trigger(self, log=""):
        cur_time = int(time.time())
        if cur_time > self.pre_time + self.duration:
            print(log)
            self.pre_time = cur_time


class SocketChecker(object):
    def __init__(self, host, port, topic_name, print_data, data_type=None):
        self.data = None
        self.previous_data = None
        self.print_data = print_data
        self.data_type = data_type
        self.topic_name = topic_name
        self._init_connection(host, port)

    def _init_connection(self, host, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.SUBSCRIBE, bytes(self.topic_name, "utf-8"))
        self.socket.connect("tcp://{}:{}".format(host, port))

    def _reinit_counter(self):
        self.counter = 0
        self.start_time = time.time()

    def _calculate_frequency(self):
        return self.counter / (time.time() - self.start_time)

    def _decode_array(self):
        processed_data = self.data.lstrip(bytes("{} ".format(self.topic_name), "utf-8"))
        print(pickle.loads(processed_data))

    def _decode_rgb_image(self):
        frame = self.data.lstrip(b"rgb_image ")
        encoded_data = np.fromstring(base64.b64decode(frame), np.uint8)
        image = cv2.imdecode(encoded_data, 1)
        cv2.imshow(image)
        cv2.waitKey(1)

    def check_connection(self):
        self._reinit_counter()
        while True:
            self.data = self.socket.recv()
            if self.data is not None and self.data is not self.previous_data:
                # To see the data - usually reduces the actual frequency. Use it to just see the stream
                if self.print_data:
                    if self.data_type == "FloatArray":
                        self._decode_array()
                    else:
                        self._decode_rgb_image()

                self.counter += 1
                print("Frequency: {}".format(self._calculate_frequency()))

                if self.counter > 10:
                    self._reinit_counter()
            else:
                self.start_time = time.time()
