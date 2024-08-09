from src.components.robot.xarm import Xarm

robot = Xarm("10.177.63.209")

print(robot.get_cartesian_position())
