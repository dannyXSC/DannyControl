# # Python 2/3 compatibility imports
# from __future__ import print_function
# from six.moves import input

from .robot import RobotWrapper
from src.data.eef_state import CartState, JointState, State
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial.transform import Rotation as R

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import franka_gripper.msg
from franka_msgs.srv import (
    SetForceTorqueCollisionBehavior,
    SetForceTorqueCollisionBehaviorRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    TransformStamped,
    Vector3,
)
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Franka(RobotWrapper):
    def __init__(self):
        super().__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(
            "move_group_python_interface", anonymous=True, disable_signals=True
        )
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander("panda_arm")
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        move_group.set_goal_position_tolerance(0.001)
        move_group.set_goal_orientation_tolerance(0.01)
        move_group.allow_replanning(False)
        move_group.set_planning_time(5)
        move_group.set_max_velocity_scaling_factor(1)
        move_group.set_max_acceleration_scaling_factor(0.5)
        # self.set_collision_behavior()
        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher

    @property
    def name(self):
        return "xarm"

    def if_shutdown(self):
        # TODO: Implement this
        return False

    def get_joint_state(self):
        return JointState(self.move_group.get_current_joint_values())

    def get_cartesian_position(self) -> CartState:
        eef = self.move_group.get_current_pose().pose
        position = [eef.position.x, eef.position.y, eef.position.z]
        quaternion = [
            eef.orientation.x,
            eef.orientation.y,
            eef.orientation.z,
            eef.orientation.w,
        ]
        return CartState(position=position, orientation=quaternion)

    def recorder_functions(self, use_cart=True) -> State:
        if use_cart:
            return self.get_cartesian_position()
        else:
            return self.get_joint_state()

    def move(self, joint_state: JointState, velocity_scale=0.4):
        input_angles = joint_state.to_list()
        self.move_group.set_max_velocity_scaling_factor(velocity_scale)
        self.move_group.set_joint_value_target(input_angles)
        self.move_group.go(input_angles, wait=True)
        self.move_group.stop()

    def move_coords(self, input_coords: CartState, velocity_scale=0.4):
        # input_coords: [x,y,z,quaternion]
        trans = input_coords.position
        quat = input_coords.orientation
        Pose_xyzw = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        Point_xyz = Point(x=trans[0], y=trans[1], z=trans[2])

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = Pose_xyzw
        pose_goal.position = Point_xyz
        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_max_velocity_scaling_factor(velocity_scale)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def stop(self):
        # TODO: Implement this
        pass


if __name__ == "__main__":
    robot = Franka()
    import time

    def test_print():
        while True:
            start_time = time.time()  # 记录循环开始时间
            cur_eef = robot.get_cartesian_position()
            cur_eef.position[2] += 0.1
            # robot.move_coords(cur_eef)
            print(cur_eef)
            cur_eef.position[2] -= 0.1
            # robot.move_coords(cur_eef)
            print(cur_eef)
            end_time = time.time()  # 记录循环结束时间
            loop_time = end_time - start_time  # 计算循环执行时间
            frequency = 1 / loop_time  # 计算循环频率（Hz）

            print(f"Frequency: {frequency:.2f} Hz")

    test_print()

    def test_joint():
        while True:
            cur_joint = robot.get_joint_state()
            print(cur_joint)
            cmd = input()
            if cmd == "q":
                cur_joint.joint_angles[0] -= 0.1
            elif cmd == "e":
                cur_joint.joint_angles[0] += 0.1
            elif cmd == "w":
                cur_joint.joint_angles[1] += 0.1
            elif cmd == "s":
                cur_joint.joint_angles[1] -= 0.1
            elif cmd == "a":
                cur_joint.joint_angles[2] -= 0.1
            elif cmd == "d":
                cur_joint.joint_angles[2] += 0.1
            else:
                break
            robot.move(cur_joint)

    # test_joint()

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
