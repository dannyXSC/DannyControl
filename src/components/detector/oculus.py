from src.constants import (
    VR_FREQ,
)
from src.components import Component
from src.utils.timer import FrequencyTimer
from src.utils.network import (
    create_pull_socket,
    ZMQKeypointPublisher,
    ZMQButtonFeedbackSubscriber,
)


class OculusVRHandDetector(Component):
    def __init__(self, host, oculus_port, keypoint_pub_port, is_right=True, log=False):
        self.notify_component_start(
            f"vr detector, hand {'right' if is_right else 'left'}"
        )
        self.log = log
        self.is_right = is_right

        # Initializing the network socket for getting the raw hand keypoints depands on is_right flag
        self.raw_keypoint_socket = create_pull_socket(host, oculus_port)

        if self.log:
            print(host, oculus_port)

        # ZMQ Keypoint publisher
        self.hand_keypoint_publisher = ZMQKeypointPublisher(
            host=host, port=keypoint_pub_port
        )
        self.timer = FrequencyTimer(VR_FREQ)

    # Function to process the data token received from the VR
    def _process_data_token(self, data_token):
        return data_token.decode().strip()

    # Function to Extract the Keypoints from the String Token sent by the VR
    def _extract_data_from_token(self, token):
        data = self._process_data_token(token)
        information = dict()
        keypoint_vals = [0] if data.startswith("absolute") else [1]
        # Data is in the format <hand>:x,y,z|x,y,z|x,y,z
        vector_strings = data.split(":")[1].strip().split("|")
        for vector_str in vector_strings:
            vector_vals = vector_str.split(",")
            for float_str in vector_vals[:3]:
                keypoint_vals.append(float(float_str))

        information["keypoints"] = keypoint_vals
        return information

    # Function to Publish the transformed Keypoints
    def _publish_data(self, keypoint_dict):
        if self.is_right:
            topic_name = "right"
        else:
            topic_name = "left"
        self.hand_keypoint_publisher.pub_keypoints(
            keypoint_array=keypoint_dict["keypoints"], topic_name=topic_name
        )

    # Function to Stream the Keypoints
    def stream(self):
        while True:
            try:
                self.timer.start_loop()
                if self.log:
                    print("wait")
                # Getting the raw keypoints
                raw_keypoints = self.raw_keypoint_socket.recv()
                # Processing the keypoints and publishing them
                keypoint_dict = self._extract_data_from_token(raw_keypoints)

                # Publish Data
                self._publish_data(keypoint_dict)
                self.timer.end_loop()
            except:
                break

        self.raw_keypoint_socket.close()
        self.hand_keypoint_publisher.stop()

        print("Stopping the oculus keypoint extraction process.")
