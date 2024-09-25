from src.constants import (
    VR_FREQ,
)
from src.components import Component
from src.utils.timer import FrequencyTimer
from src.utils.network import (
    create_pull_socket,
    ZMQKeypointPublisher,
    create_response_socket,
)


# This class is used to detect the hand keypoints from the VR and publish them.
class OculusVRTwoHandDetector(Component):
    def __init__(
        self,
        host,
        oculus_right_port,
        oculus_left_port,
        keypoint_pub_port,
    ):
        self.notify_component_start("vr detector")
        # Initializing the network socket for getting the raw keypoints
        self.raw_keypoint_right_socket = create_pull_socket(host, oculus_right_port)
        # self.teleop_reset_socket = create_pull_socket(host, teleop_reset_port)
        self.raw_keypoint_left_socket = create_pull_socket(host, oculus_left_port)

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

    # Function to Publish the right hand transformed Keypoints
    def _publish_right_data(self, keypoint_dict):
        self.hand_keypoint_publisher.pub_keypoints(
            keypoint_array=keypoint_dict["keypoints"], topic_name="right"
        )

    # Function to Publish the left hand transformed Keypoints
    def _publish_left_data(self, keypoint_dict):
        self.hand_keypoint_publisher.pub_keypoints(
            keypoint_array=keypoint_dict["keypoints"], topic_name="left"
        )

    # Function to publish the left/right hand keypoints and button Feedback
    def stream(self):

        while True:
            try:
                self.timer.start_loop()

                # Getting the raw keypoints
                raw_right_keypoints = self.raw_keypoint_right_socket.recv()
                raw_left_keypoints = self.raw_keypoint_left_socket.recv()

                # Processing the keypoints and publishing them
                keypoint_right_dict = self._extract_data_from_token(raw_right_keypoints)
                keypoint_left_dict = self._extract_data_from_token(raw_left_keypoints)
                self._publish_right_data(keypoint_right_dict)
                self._publish_left_data(keypoint_left_dict)
                self.timer.end_loop()

            except KeyboardInterrupt:
                break

        self.raw_keypoint_right_socket.close()
        self.raw_keypoint_left_socket.close()
        self.hand_keypoint_publisher.stop()
        print("Stopping the oculus keypoint extraction process.")
