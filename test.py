from src.components.robot.xarm import Xarm

robot = Xarm("10.177.70.209")

print(robot.get_cartesian_position())