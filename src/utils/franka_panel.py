from src.components.robot.franka import Franka
from src.components.robot.franka_gripper import FrankaGripper
from src.data.eef_state import CartState, JointState
from src.constants import *
from src.utils.timer import FrequencyTimer

import time
import pygame
import numpy as np


class FrankaPanel:
    def __init__(self):
        self.robot = Franka()
        self.freq = PANEL_FREQ
        self.timer = FrequencyTimer(RECORD_FREQ)

    def _get_infos(self):
        return dict(
            joint_state=self.robot.get_joint_state(),
            cart_state=self.robot.get_cartesian_position(),
        )

    def show(self):
        while True:
            self.timer.start_loop()
            infos = self._get_infos()
            joint_state = (self.robot.get_joint_state(),)
            cart_state = (self.robot.get_cartesian_position(),)

            print("\033[H\033[J", end="")  # 清屏
            print("Robot Status:")
            print(f"{joint_state=}")
            print(f"{cart_state=}")

            self.timer.end_loop()


class FrankaPanelViewer:
    WIDTH, HEIGHT = 1200, 600
    # 定义颜色
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GRAY = (200, 200, 200)
    BACKGROUND_ARRAY = (240, 240, 240)  # 灰色背景
    BACKGROUND_ARRAY2 = (200, 200, 255)  # 蓝色背景
    BACKGROUND_INT_VALUE = (200, 255, 200)  # 绿色背景

    class Button:
        def __init__(self, x, y, text, action, width=100, height=50):
            self.width = width
            self.height = height
            self.rect = pygame.Rect(x, y, width, height)
            self.text = text
            self.action = action
            self.button_font = pygame.font.SysFont("Arial", 30)

        def draw(self, screen):
            pygame.draw.rect(screen, (200, 200, 200), self.rect)
            text_surface = self.button_font.render(self.text, True, (0, 0, 0))
            screen.blit(
                text_surface,
                (
                    self.rect.x + (self.width - text_surface.get_width()) // 2,
                    self.rect.y + (self.height - text_surface.get_height()) // 2,
                ),
            )

        def is_pressed(self, pos):
            return self.rect.collidepoint(pos)

    def __init__(self):
        self.robot = FrankaGripper()
        self.robot.move(
            JointState(
                [
                    -4.29251469e-04,
                    -7.75564400e-01,
                    1.70081349e-03,
                    -2.35577936e00,
                    2.32602405e-04,
                    1.57109480e00,
                    7.85885839e-01,
                ]
            )
        )

        """
        pygame setting
        """
        pygame.init()
        # 设置窗口尺寸
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Franka Panel")
        self.font = pygame.font.SysFont("Arial", 20)
        self.clock = pygame.time.Clock()

        # 按钮列表
        buttons_up = []
        buttons_down = []
        buttons_up2 = []
        buttons_down2 = []
        buttons_up_int = []
        buttons_down_int = []

        delta_cart = 0.01
        delta_joints = 0.1
        delta_gripper = 1

        # 为 array 创建按钮
        for i in range(len(self.cart_state)):
            buttons_up.append(
                self.Button(
                    100 + i * 120,
                    150,
                    "+",
                    self.__build_modify_value("cart", i, delta_cart),
                )
            )
            buttons_down.append(
                self.Button(
                    100 + i * 120,
                    250,
                    "-",
                    self.__build_modify_value("cart", i, -delta_cart),
                )
            )

        # 为 array2 创建按钮
        for i in range(len(self.joint_state)):
            buttons_up2.append(
                self.Button(
                    100 + i * 120,
                    350,
                    "+",
                    self.__build_modify_value("joint", i, delta_joints),
                )
            )
            buttons_down2.append(
                self.Button(
                    100 + i * 120,
                    450,
                    "-",
                    self.__build_modify_value("joint", i, -delta_joints),
                )
            )

        # 为 int_value 创建按钮
        buttons_up_int.append(
            self.Button(
                100 + len(self.cart_state) * 120 + 20,
                150,
                "+",
                self.__build_modify_value("gripper", 0, delta_gripper),
            )
        )
        buttons_down_int.append(
            self.Button(
                100 + len(self.cart_state) * 120 + 20,
                250,
                "-",
                self.__build_modify_value("gripper", 0, -delta_gripper),
            )
        )

        self.buttons_up = buttons_up
        self.buttons_down = buttons_down
        self.buttons_up2 = buttons_up2
        self.buttons_down2 = buttons_down2
        self.buttons_up_int = buttons_up_int
        self.buttons_down_int = buttons_down_int
        self.running = True

    def __build_modify_value(self, type, index, delta):
        def debug(info):
            print(f"{info=}")

        def modify_cart(pos):
            cart_state = self.cart_state
            cart_state[index] = cart_state[index] + delta
            final_state = CartState(position=cart_state[:3], orientation=cart_state[3:])
            debug(final_state)
            self.robot.move_coords(final_state)

        def modify_joint(pos):
            joint_state = self.joint_state
            joint_state[index] += delta
            final_state = JointState(joint_state)
            debug(final_state)
            self.robot.move(final_state)

        def modify_gripper(pos):
            gripper_state = max(min(self.gripper_percentage + delta, 1.0), 0.0)
            self.robot.move_gripper_percentage(gripper_state, force=1.0)
            debug(gripper_state)

        if type == "cart":
            return modify_cart
        elif type == "joint":
            return modify_joint
        elif type == "gripper":
            return modify_gripper

    @property
    def cart_state(self):
        return self.robot.get_cartesian_position().to_list()

    @property
    def joint_state(self):
        return self.robot.get_joint_state().to_list()

    @property
    def gripper_state(self):
        return self.robot.get_gripper_state().to_list()

    @property
    def gripper_percentage(self):
        return self.robot.get_gripper_percentage()

    def show(self):
        while self.running:
            screen = self.screen
            array = self.cart_state
            array2 = self.joint_state
            int_value = self.gripper_percentage

            screen.fill(self.WHITE)

            # 绘制背景框
            pygame.draw.rect(
                screen, self.BACKGROUND_ARRAY, (80, 130, 120 * len(array) + 20, 200)
            )  # array部分背景
            pygame.draw.rect(
                screen, self.BACKGROUND_ARRAY2, (80, 330, 120 * len(array2) + 20, 200)
            )  # array2部分背景
            pygame.draw.rect(
                screen,
                self.BACKGROUND_INT_VALUE,
                (100 + len(array) * 120 + 20, 150, 100, 50),
            )  # int_value部分背景

            # 事件处理
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # 判断按钮点击
                    for button in (
                        self.buttons_up
                        + self.buttons_down
                        + self.buttons_up2
                        + self.buttons_down2
                        + self.buttons_up_int
                        + self.buttons_down_int
                    ):
                        if button.is_pressed(event.pos):
                            button.action(event.pos)

            # 显示 array 内容
            for i, value in enumerate(array):
                text_surface = self.font.render(f"{value:.3f}", True, self.BLACK)
                screen.blit(text_surface, (100 + i * 120, 200))

            # 显示 array2 内容
            for i, value in enumerate(array2):
                text_surface = self.font.render(f"{value:.3f}", True, self.BLACK)
                screen.blit(text_surface, (100 + i * 120, 400))  # 调整到按钮之间

            # 显示 int_value 内容
            text_surface = self.font.render(f"{int_value:.1f}", True, self.BLACK)
            screen.blit(
                text_surface, (100 + len(array) * 120 + 20, 200)
            )  # int_value 显示在 array 右边

            # 绘制 array 按钮
            for button in self.buttons_up:
                button.draw(screen)
            for button in self.buttons_down:
                button.draw(screen)

            # 绘制 array2 按钮
            for button in self.buttons_up2:
                button.draw(screen)
            for button in self.buttons_down2:
                button.draw(screen)

            # 绘制 int_value 按钮
            for button in self.buttons_up_int:
                button.draw(screen)
            for button in self.buttons_down_int:
                button.draw(screen)

            pygame.display.flip()
            self.clock.tick(10)

        pygame.quit()


if __name__ == "__main__":
    panel = FrankaPanelViewer()
    panel.show()
    # panel.robot.move_coords(
    #     CartState(
    #         position=[0.29221009, 0.0248221, 0.61537393],
    #         orientation=[0.92653802, -0.37416551, -0.02456518, 0.03039786],
    #     )
    # )
