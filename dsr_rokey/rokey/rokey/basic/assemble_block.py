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
from DSR_ROBOT2 import set_digital_output, get_digital_input, wait

def grip():
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    # 그리퍼 닫힘
    time.sleep(1)

def release():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    # 그리퍼 닫힘
    time.sleep(1)

def main(args=None):
    try: # DSR 언어, 인자 import
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            trans,
            set_ref_coord,
            DR_BASE, wait,
            task_compliance_ctrl,
            set_desired_force, DR_FC_MOD_REL,
            check_force_condition, release_force, release_compliance_ctrl, DR_AXIS_Z, mwait
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    set_ref_coord(DR_BASE)

    # 포즈 지정
    pos_home = posj([0, 0, 90, 0, 90, 0])
    block_1_grip = posx(426.39, 215.03 ,43.02, 25.45, -176.82, 24.56 ) # 블록 잡을 위치 지정
    # block_2_grip = posx() # 블록 잡을 위치 지정
    
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok(): # 구동부
        release()
        movej(pos_home, vel = 60, acc = 60)
        wait(0.5)

        movel(block_1_grip, vel = 60, acc = 60)
        wait(0.5)

        block_1_grip_down = [0, 0, -40, 0, 0, 0]
        mwait()
        block_1_grip_up = [0, 0, 100, 0, 0, 0]
        mwait() 
        
        movel(block_1_grip_down, vel=60, acc = 60, mod = 1)
        print("movel")
        print(block_1_grip_down)
        wait(0.5)
        grip()
        wait(0.5)
        movel(block_1_grip_up, vel = 60, acc = 60, mod = 1)

        # Assemble_1
        target1 = posx(485.38,115.14 ,15.38, 12.14, -179.19, 11.50) # 블럭 놓을 위치의 위로 이동
        movel(target1, vel=60, acc=60)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(3)
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(3)
        force_condition = check_force_condition(DR_AXIS_Z, max=20)
        time.sleep(1)
        while force_condition == 0: # 힘제어로 블럭 놓기
            force_condition = check_force_condition(DR_AXIS_Z, max=20)
            print(force_condition)
            time.sleep(1)

        release_force()
        time.sleep(1)
        release_compliance_ctrl()
        time.sleep(1)
        release()
        time.sleep(1)
        target1_up = [0, 0, 100, 0, 0, 0]
        movel(target1_up, vel=60, acc=60, mod = 1),
    

    # movej(pos_home, vel = 60, acc = 60)
    # time.sleep(1)

    rclpy.shutdown()
if __name__ == "__main__":
    main()
