from src.components import Component
from src.utils.timer import FrequencyTimer
from src.constants import *
from src.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from src.components.robot.xarm import Xarm
from src.data.xarm import XarmInfo


class XarmInfoNotifier(Component):
    def __init__(self, host, port, xarm_ip):
        self.notify_component_start("xarm notifier")

        self.xarm_publisher = ZMQKeypointPublisher(host, port)
        self.timer = FrequencyTimer(TRANS_FREQ)

        self.robot = Xarm(xarm_ip)

    def stream(self):
        while True:
            if self.robot.if_shutdown():
                continue

            try:
                self.timer.start_loop()

                xarm_info = (
                    XarmInfo()
                    .set_joint_angle(self.robot.get_joint_position())
                    .set_end_position(self.robot.get_cartesian_position())
                    .set_gripper_state(self.robot.get_gripper_state())
                )

                payload = xarm_info.to_json()

                self.xarm_publisher.pub_keypoints(
                    payload, topic_name=XARM_NOTIFIER_TOPIC
                )

                # End the timer
                self.timer.end_loop()
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
