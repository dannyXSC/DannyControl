from .base import BaseInfo


class XarmInfo(BaseInfo):
    def __init__(self, joint_angle=None, end_position=None):
        self.set_joint_angle(joint_angle)
        self.set_end_position(end_position)

    def set_joint_angle(self, joint_angle):
        self.joint_angle = joint_angle
        return self

    def set_end_position(self, end_position):
        self.end_position = end_position
        return self

    def to_json(self):
        return dict(joint_angle=self.joint_angle, end_position=self.end_position)

    def from_json(self, js):
        self.joint_angle = js["joint_angle"]
        self.end_position = js["end_position"]
