speed = 1000
wait = False
wait_motion = True

with open("./log.txt", "r") as f:
    for line in f:
        target = eval(line)
        robot.move_coords(
            input_coords=target,
            wait=wait,
            wait_motion=wait_motion,
            speed=speed,
        )
