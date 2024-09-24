from src.utils.network import *
import time
import traceback


def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.connect("tcp://10.162.189.83:8087")
    while True:
        try:
            topic_name = "right_hand"
            buffer = pickle.dumps([0, 1, 0, 0, 1, 0, 0, 1, 0, 0], protocol=-1)
            socket.send(bytes("{} ".format(topic_name), "utf-8") + buffer)
            print(buffer)
            time.sleep(1)
        except:
            traceback.print_exc()
            break


main()
