# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
node = rclpy.create_node("assemble_block_node", namespace=ROBOT_ID)

DR_init.__dsr__node = node
from DSR_ROBOT2 import set_digital_output, get_digital_input, amovel, movel

def grip():
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(1)

def grip():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(1)

def grid_search_from_start(start_pose, x_max=450, y_max=600, x_step=50, y_step=50):
    """
    start_pose에서 시작하여 직사각형 영역을 지그재그 방식으로 상대 이동하며 탐색합니다.

    - start_pose: posx([x, y, z, rx, ry, rz]) 형태의 시작 포즈
    - x_max, y_max: 탐색 범위(mm)
    - x_step, y_step: 이동 간격(mm)
    """

    current_x = 0.0
    current_y = 0.0
    row_index = 0

    import math
    total_x_moves = math.ceil(x_max / x_step)
    total_y_moves = math.ceil(y_max / y_step)

    while current_y <= y_max:
        if row_index % 2 == 0:
            # 짝수 줄: 왼쪽 → 오른쪽 (+x)
            while current_x < x_max:
                dx = min(x_step, x_max - current_x)
                amovel([dx, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)
                print(f"[이동] Δx={dx}, Δy=0")
                current_x += dx
        else:
            # 홀수 줄: 오른쪽 → 왼쪽 (-x)
            while current_x > 0:
                dx = min(x_step, current_x)
                amovel([-dx, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)
                print(f"[이동] Δx={-dx}, Δy=0")
                current_x -= dx

        # 다음 줄로 y 방향 이동
        current_y += y_step
        row_index += 1
        if current_y <= y_max:
            movel([0, -y_step, 0, 0, 0, 0], vel=60, acc=60, mod=1)
            print(f"[줄 이동] Δx=0, Δy={y_step}")

    print("탐색 완료")



def main(args=None):
    try: # DSR 언어, 인자 import
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            trans,
            set_ref_coord,
            DR_BASE
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    set_ref_coord(DR_BASE)

    # 포즈 지정
    JReady = [0, 0, 90, 0, 90, 0]
    start_pose = posx([162.30, 305.92, 100.00, 53.48, -179.14, 53.20])
    # current_pose = start_pose

    # delta_list = [
    #     [0, 450, 0, 0, 0, 0],
    #     [-10, 0, 0, 0, 0, 0],
    #     [0, -450, 0, 0, 0, 0]
    # ]

    
    
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok(): # 구동부

        print("movej")
        movej(JReady, vel=VELOCITY, acc=ACC)
        movel(start_pose, vel=VELOCITY, acc=ACC)
        grid_search_from_start(start_pose)
        
    

    rclpy.shutdown()
if __name__ == "__main__":
    main()
