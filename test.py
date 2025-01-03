from src.components.robot.xarm import Xarm

robot = Xarm("10.177.63.209")
# robot.move_coords([160, 400, 123, -90, 90, 0])
robot.move_coords([151.2, 312.3, 123, -90, 90, 0])
robot.move_coords([251.2, 312.3, 120, -90, 90, 0])
robot.move_coords([251.2, 412.3, 120, -90, 90, 0])