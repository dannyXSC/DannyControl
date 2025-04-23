import rospy
import franka_gripper.msg
from sensor_msgs.msg import JointState
import sys

# Brings in the SimpleActionClient
import actionlib

from .franka import Franka
from src.data.eef_state import GripperState, RobotState
from src.constants import *


class FrankaGripper(Franka):
    def __init__(self):
        super().__init__()
        # rospy.init_node("franka_gripper", anonymous=True, disable_signals=True)

        self.MAX_WIDTH = 0.08

        self._client = actionlib.SimpleActionClient(
            "/franka_gripper/grasp", franka_gripper.msg.GraspAction
        )
        self._open_client = actionlib.SimpleActionClient(
            "/franka_gripper/move", franka_gripper.msg.MoveAction
        )

        self.gripper_sub = rospy.Subscriber(
            "/franka_gripper/joint_states", JointState, self.__joint_state_callback
        )
        self.state = None
        while self.state is None:
            pass

        self._client.wait_for_server()
        self._data_frequency = rospy.Rate(TRANS_FREQ)

    def __joint_state_callback(self, data):
        pos = data.position
        dis = pos[0] + pos[1]
        if self.state is None:
            self.state = GripperState(dis)
        else:
            self.state.set(dis)

    @property
    def name(self):
        return "franka_gripper"

    def recorder_functions(self, use_cart=True) -> RobotState:
        arm_state = super().recorder_functions(use_cart)
        gripper_state = self.get_gripper_state()
        return RobotState(arm_state=arm_state, eef_state=gripper_state)

    def get_gripper_state(self) -> GripperState:
        return self.state

    def get_gripper_percentage(self):
        return self.state.to_list()[0] / self.MAX_WIDTH

    def move_gripper(self, width, speed=0.4, force=120):
        cur_width = self.get_gripper_state().to_list()[0]
        if width < cur_width:
            self._grasp(width, speed, force)
        else:
            self._open(width, speed)

    def _grasp(self, width, speed=0.4, force=120):
        goal = franka_gripper.msg.GraspGoal(width=width, speed=speed, force=force)
        goal.epsilon.inner = 0.05
        goal.epsilon.outer = 0.05
        self._client.send_goal(goal)
        # r = self._client.wait_for_result()

    def _open(self, width, speed=0.4):
        goal = franka_gripper.msg.MoveGoal(width=width, speed=speed)
        self._open_client.send_goal(goal)
        # r = self._open_client.wait_for_result()

    def move_gripper_percentage(self, percentage, speed=0.4, force=20):
        self.move_gripper(percentage * self.MAX_WIDTH, speed, force=force)

    def if_shutdown(self):
        super().if_shutdown()

    def stop(self):
        # self._client.cancel_goal()
        self.gripper_sub.unregister()
        super().stop()


if __name__ == "__main__":
    robot = FrankaGripper()
    import time

    def test_gripper():
        while True:
            start_time = time.time()  # 记录循环开始时间
            robot.move_gripper(0.1, 0.1)
            print(robot.recorder_functions().to_list())
            robot.move_gripper(0, 0.1)
            print(robot.recorder_functions().to_list())
            end_time = time.time()  # 记录循环结束时间
            loop_time = end_time - start_time  # 计算循环执行时间
            frequency = 1 / loop_time  # 计算循环频率（Hz）

            print(f"Frequency: {frequency:.2f} Hz")

    def test_print():
        while True:
            start_time = time.time()  # 记录循环开始时间
            cur_eef = robot.get_cartesian_position()
            # cur_eef.position[2] += 0.1
            # robot.move_coords(cur_eef, 1)
            print(cur_eef)
            # cur_eef.position[2] -= 0.1
            # robot.move_coords(cur_eef, 1)
            print(cur_eef)
            end_time = time.time()  # 记录循环结束时间
            loop_time = end_time - start_time  # 计算循环执行时间
            frequency = 1 / loop_time  # 计算循环频率（Hz）

            print(f"Frequency: {frequency:.2f} Hz")

    # test_print()

    def test_joint():
        while True:
            cur_joint = robot.get_joint_state()
            print(cur_joint)
            # cmd = input()
            # if cmd == "q":
            #     cur_joint.joint_angles[0] -= 0.1
            # elif cmd == "e":
            #     cur_joint.joint_angles[0] += 0.1
            # elif cmd == "w":
            #     cur_joint.joint_angles[1] += 0.1
            # elif cmd == "s":
            #     cur_joint.joint_angles[1] -= 0.1
            # elif cmd == "a":
            #     cur_joint.joint_angles[2] -= 0.1
            # elif cmd == "d":
            #     cur_joint.joint_angles[2] += 0.1
            # else:
            #     break
            # robot.move(cur_joint)

    test_joint()

    def test_cart():
        while True:
            cur_eef = robot.get_cartesian_position()
            print(cur_eef)
            cmd = input()
            if cmd == "q":
                cur_eef.position[2] -= 0.1
            elif cmd == "e":
                cur_eef.position[2] += 0.1
            elif cmd == "w":
                cur_eef.position[0] += 0.1
            elif cmd == "s":
                cur_eef.position[0] -= 0.1
            elif cmd == "a":
                cur_eef.position[1] -= 0.1
            elif cmd == "d":
                cur_eef.position[1] += 0.1
            else:
                break
            robot.move_coords(cur_eef)
