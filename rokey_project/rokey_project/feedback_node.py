import rclpy
from rclpy.node import Node
from rokey_interfaces.msg import TaskState
import os
from time import sleep

import RPi_I2C_driver

# from ament_index_python.packages import get_package_share_directory
# package_path = get_package_share_directory("rokey_projecy")
# os.path.join(
#     package_path, 'resource', 'T_gripper2camera.npy'
# )

[tumblr, wallet, gum, snack, cardkey]

class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback_node')

        # LCD 초기화
        self.lcd = RPi_I2C_driver.lcd()
        self.lcd.lcd_clear()
        self.lcd.backlight(1)

        # 커스텀 문자 등록
        self.load_custom_chars()

        # 토픽 구독
        self.sub = self.create_subscription(TaskState, '/task_state', self.task_callback, 10)
        self.get_logger().info("feedback_node (LCD + audio) 시작됨")

    def task_callback(self, msg: TaskState):
        state = msg.state
        obj = msg.object_name

        self.lcd.lcd_clear()

        if state == "start":
            self.lcd.lcd_display_string("Hello!", 1)
            self.lcd.lcd_display_string("Have a nice day!", 2)
            self.play_audio("Have_a_nice_day.wav")

        elif state == "good_night": # good night
            self.lcd.lcd_display_string(f"good night!", 1)
            self.lcd.lcd_display_string(f"have a sweat dream~", 2)
            self.play_audio(f"good_night.wav")

        elif state == "grapped": # detect items 쫙
            self.lcd.lcd_display_string(f"grapped object:", 1)
            self.lcd.lcd_display_string(f"{obj}", 2)
            self.play_audio(f"{obj}.wav")

        elif state == "placed": # detect placed items 쫙
            self.lcd.lcd_display_string(f"placed object:", 1)
            self.lcd.lcd_display_string(f"{obj}", 2)
            self.play_audio(f"{obj}.wav")

        elif state == "out": # detect items out 쫙
            self.lcd.lcd_display_string(f"{obj}", 1)
            self.lcd.lcd_display_string(f"out", 2)
            self.play_audio(f"{obj}.wav")

        elif state == "searching":
            self.show_progress_bar()
            self.play_audio(f"searching.wav")


    # def play_audio(self, filename):
    #     sound_dir = os.path.join(get_package_share_directory('rokey_project'), 'sounds')
    #     path = os.path.join(sound_dir, filename)

    #     self.get_logger().info(f"Trying to play: {path}")
    #     if os.path.exists(path):
    #         os.system(f"aplay \"{path}\"")
    #     else:
    #         self.get_logger().warn(f"{filename} None")

    def play_audio(self, filename):
        path = f"/home/rokey/ros2_ws/install/rokey_project/share/rokey_project/sounds/{filename}"
        if os.path.exists(path):
            os.system(f"aplay {path}")
        else:
            self.get_logger().warn(f"{path} None")

    def show_progress_bar(self):
        """
        LCD에 진행 바 애니메이션을 출력합니다.
        :param lcd: RPi_I2C_driver.lcd() 객체
        :param row: 출력할 행 (기본 1행)
        :param start_col: 시작 열 위치 (기본 6번째 칸부터)
        :param steps: 몇 칸을 채울지 (기본 2칸)
        :param delay: 각 단계 간 딜레이 (초 단위)
        """
        # 바 단계용 커스텀 문자 정의
        # bar_chars = [
        # # Char 0 - left arrow
        # [ 0x1,0x3,0x7,0xf,0xf,0x7,0x3,0x1 ],
        # # Char 1 - left one bar 
        # [ 0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10 ],
        # # Char 2 - left two bars
        # [ 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18 ],
        # # Char 3 - left 3 bars
        # [ 0x1c,0x1c,0x1c,0x1c,0x1c,0x1c,0x1c,0x1c ],
        # # Char 4 - left 4 bars
        # [ 0x1e,0x1e,0x1e,0x1e,0x1e,0x1e,0x1e,0x1e ],
        # # Char 5 - left start
        # [ 0x0,0x1,0x3,0x7,0xf,0x1f,0x1f,0x1f ],
        # # Char 6 - 
        # # [ ],
        # ]

        bar_chars = [
        [ 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 ],  # ▏
        [ 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 ],  # ▎
        [ 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C ],  # ▍
        [ 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E ],  # ▌
        ]

        block = chr(255)
        pause = 0.2

        self.lcd.lcd_display_string("searching...", 1)
        self.lcd.lcd_load_custom_chars(bar_chars)

        # 0열부터 8열까지 총 9칸에 대해 차오름 애니메이션 실행
        for col in range(9):
            for level in range(len(bar_chars)):
                self.lcd.lcd_display_string_pos(chr(level), 2, col)
                sleep(pause)
            self.lcd.lcd_display_string_pos(block, 2, col)
            sleep(pause)


    def show_smiley(self):
        """6칸짜리 웃는 얼굴 출력"""
        self.lcd.lcd_load_custom_chars(self.fontdata1)
        self.lcd.lcd_display_string_pos(chr(0), 1, 9)
        self.lcd.lcd_display_string_pos(chr(1), 1,10)
        self.lcd.lcd_display_string_pos(chr(2), 1,11)
        self.lcd.lcd_display_string_pos(chr(3), 2, 9)
        self.lcd.lcd_display_string_pos(chr(4), 2,10)
        self.lcd.lcd_display_string_pos(chr(5), 2,11)

    def load_custom_chars(self):
        """이모지/게이지 문자 정의"""
        # 웃는 얼굴 구성 문자 (6개)
        self.fontdata1 = [
            [ 0x00,0x00,0x03,0x04,0x08,0x19,0x11,0x10 ],
            [ 0x00,0x1F,0x00,0x00,0x00,0x11,0x11,0x00 ],
            [ 0x00,0x00,0x18,0x04,0x02,0x13,0x11,0x01 ],
            [ 0x12,0x13,0x1b,0x09,0x04,0x03,0x00,0x00 ],
            [ 0x00,0x11,0x1f,0x1f,0x0e,0x00,0x1F,0x00 ],
            [ 0x09,0x19,0x1b,0x12,0x04,0x18,0x00,0x00 ],
        ]
        self.lcd.lcd_load_custom_chars(self.fontdata1)

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
