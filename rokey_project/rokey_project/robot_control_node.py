import rclpy
import DR_init
import time
from rclpy.node import Node
from rokey_interfaces.msg import TaskState

## publisher node 
class TaskStatePublisher(Node):
    def __init__(self):
        super().__init__('task_state_publisher')
        self.publisher = self.create_publisher(TaskState, '/task_state', 10)

    def publish_state(self, state, obj=""):
        msg = TaskState()
        msg.state = state
        msg.object_name = obj
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: state={state}, object={obj}")

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
node = rclpy.create_node("assemble_block_node", namespace=ROBOT_ID)
publisher = TaskStatePublisher()
DR_init.__dsr__node = node

## DSR 언어, 인자 import
try: 
    from DSR_ROBOT2 import (
        set_tool,
        set_tcp,
        movej,
        movel,
        set_ref_coord,
        DR_BASE,
        DR_TOOL,
        get_tool_force,
        get_current_posx,
        DR_AXIS_Z,
        DR_AXIS_Y,
        DR_AXIS_X,
        amovel,
        amovej,
        task_compliance_ctrl,
        set_desired_force,
        DR_FC_MOD_REL,
        check_force_condition,
        release_force,
        release_compliance_ctrl,
        set_digital_output,
        get_digital_input,
        mwait,
    )
    from DR_common2 import posx, posj
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

## 그리퍼 setting
set_tool("Tool Weight1")
set_tcp("GripperDA_v1")


'''그리퍼 조작 함수'''
def grip():
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(1.5)

def release():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(1.5)


'''로봇을 칠 때 까지 대기하는 함수'''
def robot_wait():
    print("로봇을 쳐주세요! (x방향: 꺼내기, y방향: 넣기)")
    put_cnt=0
    out_cnt=0
    if put_cnt==0 or out_cnt==0:
        put_true=False
        out_true=False
        val_y = check_force_condition(DR_AXIS_Y, 10, 30, DR_TOOL)
        val_x = check_force_condition(DR_AXIS_X, 10, 30, DR_TOOL)
        while True:
            val_y = check_force_condition(DR_AXIS_Y, 10, 30, DR_TOOL)
            val_x = check_force_condition(DR_AXIS_X, 10, 30, DR_TOOL)

            ## y 방향으로 쳤을 때 수납 시작
            if val_y == 0:
                print(f"check_force_condition Y: {val_y}, X: {val_x}")
                print("수납을 시작합니다.")
                publisher.publish_state(state = "good_night")
                rclpy.spin_once(publisher)
                inputs = get_user_inputs()
                print(f'inputs = {inputs}')
                put_true=True
                put_cnt+=1
                break

            ## x 방향으로 쳤을 때 꺼내기 시작
            elif val_x == 0:
                print(f"check_force_condition Y: {val_y}, X: {val_x}")
                print("꺼내기를 시작합니다.")
                publisher.publish_state(state = "start")
                rclpy.spin_once(publisher)
                inputs = []
                out_true=True
                out_cnt+=1
                break
    return put_true, out_true, inputs


'''물체명, 적재 위치 지정하는 함수'''
def get_user_inputs():
    print("물체명과 위치 번호를 입력하세요 (예: 'tumblr 1').\n엔터만 입력하면 종료됩니다.\n")
    inputs = []
    while True:
        line = input("넣을 물건: ").strip()
        if line == "":
            break
        inputs.append(line)
    return inputs


'''ㄹ자로 탐색하는 함수'''
def find_object(vel, acc):
    publisher.publish_state(state = "searching")
    rclpy.spin_once(publisher)
    movel([162.30, 305.92, 100.00, 53.48, -179.14, 53.20], vel=vel, acc=acc) # 탐색 초기 위치
    cnt=0
    i=1
    force_triggered=False
    #duration= 400/60=6.7
    while cnt<9 and not force_triggered:
        movel([0, -50, 0, 0, 0, 0], vel=vel, acc=acc,mod=1)
        if i%2 != 0:
            amovel([400, 0, 0, 0, 0, 0], vel=vel, acc=acc, mod=1)
            i+=1
        else:
            amovel([-400, 0, 0, 0, 0, 0], vel=vel, acc=acc, mod=1)
            i+=1
        start_time = time.time()
        while time.time() - start_time < 6.7:
            fx, fy, fz = get_tool_force()[:3]
            total_force = (fx**2 + fy**2 + fz**2)**0.5
            # print(f'total_force: {total_force:.2f} N')
            threshold_force = 15
            if total_force >= threshold_force:
                force_triggered=True
                new_pos = get_current_posx()[0]
                print(f'Force reached: {total_force:.2f} N')
                break
            time.sleep(0.1)
        cnt+=1

    return i, force_triggered, new_pos


'''force control로 물건 높이를 인식하고 집는 함수'''
def grap_object(new_pos, i, height_dict, HEIGHT_RANGE):
    #new_pos=new_pos+[-20,0,0,0,0,0]
    movel(new_pos,vel=60,acc=60,mod=0)
    movel([0, 0, 120, 0, 0, 0], vel=60, acc=60,mod=1)
    if i%2!=0:
        movel([-100,0,0,0,0,0],vel=60,acc=60,mod=1)
    else:
        movel([+100,0,0,0,0,0],vel=60,acc=60,mod=1)
    #movel([0, 0, -20, 0, 0, 0], vel=60, acc=60,mod=1)

    ## force control
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    time.sleep(1)
    set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    time.sleep(1)

    # 높이 감지
    force_condition = check_force_condition(DR_AXIS_Z, max=20)
    time.sleep(0.5)
    while force_condition == 0:
        force_condition = check_force_condition(DR_AXIS_Z, max=20)
        # print(f'force_condition = {force_condition}')
        time.sleep(0.5)
    time.sleep(0.5)
    pos = get_current_posx()[0][2]
    print(f'pos_z: {pos:.2f} mm')
    release_force()
    time.sleep(1)
    release_compliance_ctrl()
    time.sleep(1)
    
    if height_dict['tumblr'] - HEIGHT_RANGE < pos < height_dict['tumblr'] + HEIGHT_RANGE: #
        # tumblr일 때 20mm 올리고 70mm 내림
        detected_name = 'tumblr'
        print(f'detect = {detected_name}')
        movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
        release()
        movel([0, 0, -70, 0, 0, 0], vel=60, acc=60,mod=1)
        grip()
        movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

    elif height_dict['snack'] - HEIGHT_RANGE < pos < height_dict['snack'] + HEIGHT_RANGE: #
        # snack일 때 20mm 올리고 70mm 내림
        detected_name = 'snack'
        print(f'detect = {detected_name}')
        movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
        release()
        movel([0, 0, -70, 0, 0, 0], vel=60, acc=60,mod=1)
        grip()
        movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

    elif height_dict['gum'] - HEIGHT_RANGE < pos < height_dict['gum'] + HEIGHT_RANGE: #
        # gum일 때 20mm 올리고 70mm 내림
        detected_name = 'gum'
        print(f'detect = {detected_name}')
        movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
        release()
        movel([0, 0, -70, 0, 0, 0], vel=60, acc=60,mod=1)
        grip()
        movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

    # elif height_dict['양치세트'] - HEIGHT_RANGE < pos < height_dict['양치세트'] + HEIGHT_RANGE: #
    #     # 양치세트일 때 20mm 올리고 70mm 내림
    #     detected_name = '양치세트'
    #     print(f'detect = {detected_name}')
    #     movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
    #     release()
    #     movel([0, 0, -70, 0, 0, 0], vel=60, acc=60,mod=1)
    #     grip()
    #     movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

    # elif height_dict['우산'] - HEIGHT_RANGE < pos < height_dict['우산'] + HEIGHT_RANGE: #
    #     # 우산일 때 20mm 올리고 63mm 내림
    #     detected_name = '우산'
    #     print(f'detect = {detected_name}')
    #     movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
    #     release()
    #     movel([0, 0, -63, 0, 0, 0], vel=60, acc=60,mod=1)
    #     grip()
    #     movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)
    
    elif height_dict['wallet'] - HEIGHT_RANGE < pos < height_dict['wallet'] + HEIGHT_RANGE: #
        # 지갑일 때 20mm 올리고 67mm 내림
        detected_name = 'wallet'
        print(f'detect = {detected_name}')
        movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
        release()
        movel([0, 0, -67, 0, 0, 0], vel=60, acc=60,mod=1)
        grip()
        movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

    elif height_dict['cardkey'] - HEIGHT_RANGE < pos < height_dict['cardkey'] + HEIGHT_RANGE:
        # 카드키일 때 20mm 올리고 40mm 내림
        detected_name = 'wallet'
        movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
        release()
        movel([0, 0, -40, 0, 0, 0], vel=60, acc=60,mod=1)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(1)
        grip()
        release_compliance_ctrl()
        time.sleep(0.5)
        movel([0, 0, 100, 0, 0, 0], vel=60, acc=60, mod=1)
        detected_name = 'cardkey'
        print(f'detect = {detected_name}')

    publisher.publish_state(state = "grapped", obj = f'{detected_name}')
    rclpy.spin_once(publisher) 
    return pos


'''물건 높이로 물체를 판별하는 함수'''
def identify_object_by_z(pos, height_dict, tol=5):
    for name, ref_z in height_dict.items():
        if abs(pos - ref_z) <= tol:
            print(f'감지된 물체 = {name}')
            return name
    return None


'''물건 수납 동작을 하는 함수'''
def place_object(detected_name, name, index, J_grip_home, J_grip_right, J_grip_left, place_list, stacked, place_dict_posx):

    publisher.publish_state(state = "placed", obj = f'{detected_name}')
    rclpy.spin_once(publisher)  

    print(f"감지된 물체 '{detected_name}' → 사용자 입력 '{name} {index}'에 따라 이동합니다.")

    ## 물체 수납 위치에 이동
    if index >= 3:
        movej(J_grip_home, vel=30, acc=30, mod=0)
        movej(J_grip_right, vel=30, acc=30, mod=0)
    elif index < 3:
        movej(J_grip_left, vel=30, acc=30, mod=0)
    time.sleep(1)

    # 적재된 물체가 0개일 때 원래 위치로 이동
    if stacked[index-1] == 0:
        movel(place_list[index-1], vel=60, acc=60, mod=0)
        place_dict_posx[name] = place_list[index-1]
    # 적재된 물체가 1개일 때 원래 위치의 옆으로 이동
    elif stacked[index-1] == 1:
        print("이미 물체가 수납되었습니다. 물체 옆으로 수납합니다.")
        X_new_place_pos = place_list[index-1].copy()
        X_new_place_pos[0] += 130
        time.sleep(0.5)
        movel(X_new_place_pos, vel=30, acc=30, mod=0)
        place_dict_posx[name] = X_new_place_pos

    ## force control 물건 놓기
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    time.sleep(1)
    set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    time.sleep(1)
    force_condition = check_force_condition(DR_AXIS_Z, max=20)
    time.sleep(0.5)
    while force_condition == 0:
        force_condition = check_force_condition(DR_AXIS_Z, max=20)
        time.sleep(0.5)
    release_force()
    time.sleep(1)
    release_compliance_ctrl()
    time.sleep(1)
    release()
    movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

    ## home으로 이동
    print(f"{name}을 {index}에 수납 완료")
    print(f"home으로 이동합니다.")
    
    if index >= 3: 
        movej(J_grip_right, vel=30, acc=30, mod=0)
        movej(J_grip_home, vel=30, acc=30, mod=0)
    elif index < 3:
        movej(J_grip_left, vel=30, acc=30, mod=0)

    movej([0,0,90,0,90,0],vel=30, acc=30,mod=0)

    return place_dict_posx


'''물건 꺼내기 동작을 하는 함수'''
def out_object(vel, acc, place_list, place_dict, J_grip_home, J_grip_right, J_grip_left, stacked, place_dict_posx): 

    for name, index in place_dict.items():
        index = int(index)
        print(f"'{name}' 를 저장된 위치 {index}에서 가져옵니다")
        publisher.publish_state(state = "out", obj = f'{name}')
        rclpy.spin_once(publisher)

        ## 물건 위치로 move
        if index >= 3: 
            movej(J_grip_home, vel=30, acc=30, mod=0)
            movej(J_grip_right, vel=30, acc=30, mod=0)
        elif index < 3:
            movej(J_grip_left, vel=30, acc=30, mod=0)

        release()

        # X_out_pos = place_dict_posx[name].copy()
        # X_out_pos[2] += 50
        # print(f"place_dict_posx[name] = {place_dict_posx[name]}")
        # print(f"X_out_pos = {X_out_pos}")
        movel(place_dict_posx[name], vel=60, acc=60, mod=0)
        time.sleep(0.2)
        movel([0, 0, 50, 0, 0, 0], vel=60, acc=60, mod=1)
        time.sleep(0.5)
        grip()
        stacked[index-1] -= 1
        print(f"stacked = {stacked}")

        ## force control
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(1)
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(1)

        # force control 높이 감지
        force_condition = check_force_condition(DR_AXIS_Z, max=20)
        time.sleep(0.5)
        while force_condition == 0:
            force_condition = check_force_condition(DR_AXIS_Z, max=20)
            print(f'force_condition = {force_condition}')
            time.sleep(0.5)
        pos = get_current_posx()[0][2]
        print(f'pos: {pos:.2f} mm')
        release_force()
        time.sleep(1)
        release_compliance_ctrl()
        time.sleep(1)
        
        # name에 따라 물건 집기
        if name == 'tumblr': #
            # tumblr일 때 20mm 올리고 70mm 내림
            detected_name = 'tumblr'
            print(f'detect = {detected_name}')
            movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
            release()
            movel([0, 0, -70, 0, 0, 0], vel=60, acc=60,mod=1)
            grip()
            movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

        elif name == 'snack': #
            # snack일 때 20mm 올리고 70mm 내림
            detected_name = 'snack'
            print(f'detect = {detected_name}')
            movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
            release()
            movel([0, 0, -70, 0, 0, 0], vel=60, acc=60,mod=1)
            grip()
            movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

        elif name == 'gum': #
            # gum일 때 20mm 올리고 70mm 내림
            detected_name = 'gum'
            print(f'detect = {detected_name}')
            movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
            release()
            movel([0, 0, -70, 0, 0, 0], vel=60, acc=60,mod=1)
            grip()
            movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)
        
        elif name == 'wallet': #
            # 지갑일 때 20mm 올리고 67mm 내림
            detected_name = 'wallet'
            print(f'detect = {detected_name}')
            movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
            release()
            movel([0, 0, -67, 0, 0, 0], vel=60, acc=60,mod=1)
            grip()
            movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

        elif name == 'cardkey':
            # 카드키일 때 20mm 올리고 40mm 내림
            detected_name = 'cardkey'
            print(f'detect = {detected_name}')
            movel([0, 0, 20, 0, 0, 0], vel=60, acc=60,mod=1)
            release()
            movel([0, 0, -40, 0, 0, 0], vel=60, acc=60,mod=1)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            time.sleep(1)
            grip()
            release_compliance_ctrl()
            time.sleep(0.5)
            movel([0, 0, 100, 0, 0, 0], vel=60, acc=60, mod=1)
            
        ## home으로 돌아가는 경로
        if index >= 3: 
            movej(J_grip_right, vel=30, acc=30, mod=0)
            movej(J_grip_home, vel=30, acc=30, mod=0)
        elif index < 3:
            movej(J_grip_left, vel=30, acc=30, mod=0)
        
        movej([0,0,90,0,90,0], vel=vel, acc=acc)
        time.sleep(2)
        release()
    print("물체를 모두 꺼냈습니다!")

    return stacked



def main(args=None):
    ## ============================== Set values ==============================
    set_ref_coord(DR_BASE)

    VELOCITY, ACC = 60, 60

    place_dict = {}
    place_dict_posx = {}
    height_dict = {'tumblr': 197, 'snack': 96.42, 'gum': 80, 'wallet': 30, 'cardkey': 6}
    HEIGHT_RANGE = 10
    
    # 경유점 pose
    J_grip_home = [-79.50, 7.48, 71.07, 0.01, 101.23, -79.72]
    J_grip_right = [-101.82, 4.64, 74.69, -0.21, 100.05, -101.71]
    J_grip_left = [-57.26, 14.96, 62.37, -0.63, 102.34, -57.25]

    # place pose
    X_left_up_place = [187.11, -599.38, 159.21, 44.76, -178.08, 46.36]
    X_left_down_place = [193.88, -429.41, -22.41, 36.01, -178.39, 37.11]
    X_right_up_place = [-162.82, -605.87, 151.53, 7.36, -178.37, 8.57]
    X_right_down_place = [-148.12, -421.63, -33.47, 158.84, 178.27, 160.59]

    # place pose list
    place_list = [X_left_up_place, X_left_down_place, X_right_up_place, X_right_down_place]

    stacked = [0, 0, 0, 0]


    ## ============================== Motions ==============================

    while rclpy.ok(): # 구동부
        ## initial position
        print("move")
        movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
        release()
        grip()

        ## =============== 로봇 쳤을 때 동작 시작 ===============
        put_true, out_true, inputs = robot_wait()  # y 방향으로 쳤을 때 수납, x 방향으로 쳤을 때 꺼내기 시작
        
        ## =============== ㄹ자 탐색 ===============
        while put_true and inputs: # put_true와, inputs가 있을 때
            i, force_triggered, new_pos = find_object(vel=VELOCITY, acc=ACC)  # ㄹ자로 탐색하는 함수

            ## =============== 물체 감지되면 집기 ===============
            if force_triggered:
                pos = grap_object(new_pos, i, height_dict, HEIGHT_RANGE)  # force control로 물건 높이를 인식하고 집는 함수
                detected_name = identify_object_by_z(pos, height_dict)  # 물건 높이로 물체를 판별하는 함수
                
                ## =============== 물체 수납 motion ===============
                if detected_name:
                    # inputs 리스트 앞에서부터 pop하여 수납하기
                    object_pop = inputs.pop(0)
                    print(f'수납할 물건 = {object_pop}')
                    name, index = object_pop.split()
                    index = int(index)

                    if name == detected_name:
                        place_dict_posx = place_object(detected_name, name, index, J_grip_home, J_grip_right, J_grip_left, place_list, 
                                                       stacked, place_dict_posx)  # 물건 수납 동작을 하는 함수

                        # place_dict에 수납한 물건 저장
                        place_dict[detected_name] = index

                        stacked[index-1] += 1
                        
                        print("place_dict에 저장합니다")
                        print(f"place_dict = {place_dict}")
                        print(f"stacked = {stacked}")
                    else:
                        print(f"감지된 물체 '{detected_name}'는 사용자 입력에 포함되어 있지 않습니다.")
                    
                    # inputs 리스트에서 모두 pop하면 break하여 탐색 종료
                    if not inputs:
                        print("물건을 모두 수납하였습니다!")
                        break
                else:
                    print("z값으로부터 인식된 물체가 없습니다.")
            grip()

        ## =============== 물체 꺼내기 motion ===============
        if out_true:
            # 수납된 물체가 없을 때
            if not place_dict:
                print("저장된 물체가 없습니다. 먼저 물체를 감지하고 위치를 저장해주세요.")
            # 수납된 물체가 있을 때
            else:
                stacked = out_object(VELOCITY, ACC, place_list, place_dict, J_grip_home, J_grip_right, J_grip_left, 
                                     stacked, place_dict_posx)  # 물건 꺼내기 동작을 하는 함수
    rclpy.shutdown()


if __name__ == "__main__":
    main()