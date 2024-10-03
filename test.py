import threading
import keyboard


def monitor_keyboard():
    while True:
        if keyboard.is_pressed("q"):
            print("Key 'q' was pressed!")
            break
        # 处理其他按键
        if keyboard.is_pressed("a"):
            print("Key 'a' was pressed!")


# 在子线程中监听键盘
keyboard_thread = threading.Thread(target=monitor_keyboard)
keyboard_thread.start()

# 主线程继续其他任务
while keyboard_thread.is_alive():
    print("Main thread is running other tasks...")
    keyboard_thread.join(1)  # 非阻塞式等待
