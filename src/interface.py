import pygame
import sys
import rclpy as rp
from std_msgs.msg import Int16MultiArray, Bool, Float64MultiArray
import time
import dynamixel_sdk as dxl

# 초기화
pygame.init()
rp.init()

destination1 = "mac room"
destination2 = "i-space"
destination3 = "pbl"
destination4 = "arisu"

drawer1 = {"destination" : "", "password" : ""}
drawer2 = {"destination" : "", "password" : ""}
drawer3 = {"destination" : "", "password" : ""}

stop_flag = False

def goal_callback(input_rosmsg):
    global stop_flag
    if input_rosmsg[2] == True:
        stop_flag = True
    else:
        stop_flag = False

# create node
interface = rp.create_node("interface")
pub = interface.create_publisher(Int16MultiArray, "user", 10)
go_pub = interface.create_publisher(Bool, "/go", 10)
goal_sub = interface.create_subscription(Float64MultiArray, "/xyflag", goal_callback, 10)

# 화면 크기 설정
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("MACMORNING")

# 색상 정의
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
back_color = white

# 버튼 설정
button_color = black

main_width = 200
main_height = 100
setting_rect = pygame.Rect(screen_width * 15/80 - main_width/2, screen_height * 40/60 - main_height/2, main_width, main_height)
run_rect = pygame.Rect(screen_width * 65/80 - main_width/2, screen_height * 40/60 - main_height/2, main_width, main_height)
drawer_rect = pygame.Rect(screen_width * 40/80 - main_width/2, screen_height * 40/60 - main_height/2, main_width, main_height)

drawer_width = 600
drawer_height = 100
drawer1_rect = pygame.Rect(screen_width * 40/80 - drawer_width/2, screen_height * 15/60 - drawer_height/2, drawer_width, drawer_height)
drawer2_rect = pygame.Rect(screen_width * 40/80 - drawer_width/2, screen_height * 30/60 - drawer_height/2, drawer_width, drawer_height)
drawer3_rect = pygame.Rect(screen_width * 40/80 - drawer_width/2, screen_height * 45/60 - drawer_height/2, drawer_width, drawer_height)

des_width = 200
des_height = 100
des1_rect = pygame.Rect(screen_width * 25/80 - des_width/2, screen_height * 20/60 - des_height/2, des_width, des_height)
des2_rect = pygame.Rect(screen_width * 55/80 - des_width/2, screen_height * 20/60 - des_height/2, des_width, des_height)
des3_rect = pygame.Rect(screen_width * 25/80 - des_width/2, screen_height * 40/60 - des_height/2, des_width, des_height)
des4_rect = pygame.Rect(screen_width * 55/80 - des_width/2, screen_height * 40/60 - des_height/2, des_width, des_height)

num_width = 100
num_height = 100
num1_rect = pygame.Rect(screen_width * 10/80 - num_width/2, screen_height * 30/60 - num_height/2, num_width, num_height)
num2_rect = pygame.Rect(screen_width * 25/80 - num_width/2, screen_height * 30/60 - num_height/2, num_width, num_height)
num3_rect = pygame.Rect(screen_width * 40/80 - num_width/2, screen_height * 30/60 - num_height/2, num_width, num_height)
num4_rect = pygame.Rect(screen_width * 55/80 - num_width/2, screen_height * 30/60 - num_height/2, num_width, num_height)
num5_rect = pygame.Rect(screen_width * 70/80 - num_width/2, screen_height * 30/60 - num_height/2, num_width, num_height)

num6_rect = pygame.Rect(screen_width * 10/80 - num_width/2, screen_height * 45/60 - num_height/2, num_width, num_height)
num7_rect = pygame.Rect(screen_width * 25/80 - num_width/2, screen_height * 45/60 - num_height/2, num_width, num_height)
num8_rect = pygame.Rect(screen_width * 40/80 - num_width/2, screen_height * 45/60 - num_height/2, num_width, num_height)
num9_rect = pygame.Rect(screen_width * 55/80 - num_width/2, screen_height * 45/60 - num_height/2, num_width, num_height)
num0_rect = pygame.Rect(screen_width * 70/80 - num_width/2, screen_height * 45/60 - num_height/2, num_width, num_height)

cancel_rect = pygame.Rect(screen_width * 70/80 - num_width/2, screen_height * 15/60 - num_height/2, num_width, num_height)
done_rect = pygame.Rect(screen_width * 55/80 - num_width/2, screen_height * 15/60 - num_height/2, num_width, num_height)

check_width = 100
check_height = 50
yes_rect = pygame.Rect(screen_width * 25/80 - check_width/2, screen_height * 55/60 - check_height/2, check_width, check_height)
no_rect = pygame.Rect(screen_width * 55/80 - check_width/2, screen_height * 55/60 - check_height/2, check_width, check_height)

open_width = 200
open_height = 100
open_rect = pygame.Rect(screen_width * 40/80 - open_width/2, screen_height * 40/60 - open_height/2, open_width, open_height)


# writing 
write_color = back_color
title_width = 200
title_height = 100
title_rect = pygame.Rect(screen_width * 40/80 - title_width/2, screen_height * 5/60 - title_height/2, title_width, title_height)

password_rect = pygame.Rect(screen_width * 30/80 - title_width/2, screen_height * 15/60 - title_height/2, title_width, title_height)
open_count = 0
open_flag = False

# 모터 ID 설정
DXL_ID_1 = 1               
DXL_ID_5 = 5               

# 사용 포트 설정
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

# 제어 테이블 주소
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_ANGLE_LIMIT = 6
ADDR_MX_CCW_ANGLE_LIMIT = 8
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36

TORQUE_ENABLE = 1  
TORQUE_DISABLE = 0 
CW_ANGLE_LIMIT = 0
CCW_ANGLE_LIMIT = 0

cycle = 1024 * 2.0

# 패킷핸들러 및 포트핸들러 초기화
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)


def draw_button(rect, txt, color = button_color):
    pygame.draw.rect(screen, color, rect)
    font = pygame.font.Font(None, 36)
    text = font.render(txt, True, white)
    text_rect = text.get_rect(center=rect.center)
    screen.blit(text, text_rect)


def write(rect, txt, size = 36, color = black):
    pygame.draw.rect(screen, write_color, rect)
    font = pygame.font.Font(None, size)
    text = font.render(txt, True, color)
    text_rect = text.get_rect(center=rect.center)
    screen.blit(text, text_rect)


def destination_converter(destination):
    if destination == destination1:
        return 1
    elif destination == destination2:
        return 2
    elif destination == destination3:
        return 3
    elif destination == destination4:
        return 4


def draw_destination():
    draw_button(des1_rect, destination1)
    draw_button(des2_rect, destination2)
    draw_button(des3_rect, destination3)
    draw_button(des4_rect, destination4)


def draw_password():
    draw_button(num1_rect, '1')
    draw_button(num2_rect, '2')
    draw_button(num3_rect, '3')
    draw_button(num4_rect, '4')
    draw_button(num5_rect, '5')
    draw_button(num6_rect, '6')
    draw_button(num7_rect, '7')
    draw_button(num8_rect, '8')
    draw_button(num9_rect, '9')
    draw_button(num0_rect, '0')
    draw_button(cancel_rect, 'X')
    draw_button(done_rect, 'OK')
    

def draw_main():
    draw_button(setting_rect, "setting")
    draw_button(run_rect, "run")
    draw_button(drawer_rect, "open drawer")


def draw_drawer():
    draw_button(drawer1_rect, "red", (204, 51, 102))
    draw_button(drawer2_rect, "green", (102, 204, 51))
    draw_button(drawer3_rect, "blue", (51, 102, 204))


def draw_check():
    draw_button(drawer1_rect, drawer1["destination"] + ', ' + drawer1["password"], (204, 51, 102))
    draw_button(drawer2_rect, drawer2["destination"] + ', ' + drawer2["password"], (102, 204, 51))
    draw_button(drawer3_rect, drawer3["destination"] + ', ' + drawer3["password"], (51, 102, 204))
    draw_button(yes_rect, "Yes")
    draw_button(no_rect, "No")

def motor_init():


    # 포트 열기
    if portHandler.openPort():
        print("포트 열기 성공")
    else:
        print("포트 열기 실패")
        quit()

    if portHandler.setBaudRate(BAUDRATE):
        print("보드레이트 설정 성공")
    else:
        print("보드레이트 설정 실패")
        quit()

    # 모터에 토크 인가
    for motor_id in [DXL_ID_1, DXL_ID_5]:
        # packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_MX_TORQUE_ENABLE, 10)

    # 연속 회전 모드 설정
    for motor_id in [DXL_ID_1, DXL_ID_5]:
        packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_MX_CW_ANGLE_LIMIT, CW_ANGLE_LIMIT)
        packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_MX_CCW_ANGLE_LIMIT, CCW_ANGLE_LIMIT)


def open(num):
    pass
    # left = 1
    # right = 5

    # start_time = time.time()
    # print("open " + str(num))
    # moving_speed = 1024 + 400  # 시계 방향으로 회전
    # packetHandler.write2ByteTxRx(portHandler, right, ADDR_MX_MOVING_SPEED, moving_speed)
    # moving_speed = moving_speed - 1024
    # packetHandler.write2ByteTxRx(portHandler, left, ADDR_MX_MOVING_SPEED, moving_speed)

    # total_sum = 0

    # last_position, _, _ = packetHandler.read2ByteTxRx(portHandler, right, ADDR_MX_PRESENT_POSITION)

    # while True:
    #     current_position, _, _ = packetHandler.read2ByteTxRx(portHandler, right, ADDR_MX_PRESENT_POSITION)
    #     # print("cp: ",current_position)

    #     #=====
    #     step = last_position - current_position
    #     if  step < 0:
    #         step += 1024

    #     if step < 200:
    #         total_sum += step
        
    #     #=====

    #     print(current_position, total_sum)

    #     if total_sum > cycle:
    #         break

    #     if time.time() - start_time > 10:
    #         break

    #     time.sleep(0.1)  # 모니터링 주기

    #     last_position = current_position

    # # 모터 멈추기
    # packetHandler.write2ByteTxRx(portHandler, left, ADDR_MX_MOVING_SPEED, 0)
    # packetHandler.write2ByteTxRx(portHandler, right, ADDR_MX_MOVING_SPEED, 0)

def close(num):
    pass
    # global open_flag
    # open_flag = False
    # left = 1
    # right = 5

    # start_time = time.time()
    # print("close " + str(num))
    # moving_speed = 1024 + 400  # 시계 방향으로 회전
    # packetHandler.write2ByteTxRx(portHandler, left, ADDR_MX_MOVING_SPEED, moving_speed)
    # moving_speed = moving_speed - 1024
    # packetHandler.write2ByteTxRx(portHandler, right, ADDR_MX_MOVING_SPEED, moving_speed)

    # total_sum = 0

    # last_position, _, _ = packetHandler.read2ByteTxRx(portHandler, right, ADDR_MX_PRESENT_POSITION)

    # while True:
    #     current_position, _, _ = packetHandler.read2ByteTxRx(portHandler, right, ADDR_MX_PRESENT_POSITION)
    #     # print("cp: ",current_position)

    #     #=====
    #     step = current_position - last_position
    #     if  step < 0:
    #         step += 1024

    #     if step < 200:
    #         total_sum += step
        
    #     #=====

    #     print(current_position, total_sum)

    #     if total_sum > cycle:
    #         break

    #     if time.time() - start_time > 10:
    #         break

    #     time.sleep(0.1)  # 모니터링 주기

    #     last_position = current_position

    # # 모터 멈추기
    # packetHandler.write2ByteTxRx(portHandler, left, ADDR_MX_MOVING_SPEED, 0)
    # packetHandler.write2ByteTxRx(portHandler, right, ADDR_MX_MOVING_SPEED, 0)


def main():
    global drawer1, drawer2, drawer3
    phase = "main"
    password_list = []
    selected_drawer = 0
    wrong_flag = False
    destination_list = []
    driving_flag = False


    # motor_init()

    password = ''
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                portHandler.closePort()
                pygame.quit()
                sys.exit()
                
            if stop_flag == True:
                phase = "arrive"

            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                if phase == "destination":

                    if des1_rect.collidepoint(mouse_pos):
                        if selected_drawer == 1:
                            drawer1["destination"] = destination1
                        elif selected_drawer == 2:
                            drawer2["destination"] = destination1
                        elif selected_drawer == 3:
                            drawer3["destination"] = destination1
                        phase = "password"

                    elif des2_rect.collidepoint(mouse_pos):
                        if selected_drawer == 1:
                            drawer1["destination"] = destination2
                        elif selected_drawer == 2:
                            drawer2["destination"] = destination2
                        elif selected_drawer == 3:
                            drawer3["destination"] = destination2
                        phase = "password"

                    elif des3_rect.collidepoint(mouse_pos):
                        if selected_drawer == 1:
                            drawer1["destination"] = destination3
                        elif selected_drawer == 2:
                            drawer2["destination"] = destination3
                        elif selected_drawer == 3:
                            drawer3["destination"] = destination3
                        phase = "password"

                    elif des4_rect.collidepoint(mouse_pos):
                        if selected_drawer == 1:
                            drawer1["destination"] = destination4
                        elif selected_drawer == 2:
                            drawer2["destination"] = destination4
                        elif selected_drawer == 3:
                            drawer3["destination"] = destination4
                        phase = "password"
                    

                elif phase == "password" :
                    if num1_rect.collidepoint(mouse_pos):
                        password_list.append(1)
                    elif num2_rect.collidepoint(mouse_pos):
                        password_list.append(2)
                    elif num3_rect.collidepoint(mouse_pos):
                        password_list.append(3)
                    elif num4_rect.collidepoint(mouse_pos):
                        password_list.append(4)
                    elif num5_rect.collidepoint(mouse_pos):
                        password_list.append(5)
                    elif num6_rect.collidepoint(mouse_pos):
                        password_list.append(6)
                    elif num7_rect.collidepoint(mouse_pos):
                        password_list.append(7)
                    elif num8_rect.collidepoint(mouse_pos):
                        password_list.append(8)
                    elif num9_rect.collidepoint(mouse_pos):
                        password_list.append(9)
                    elif num0_rect.collidepoint(mouse_pos):
                        password_list.append(0)
                    elif cancel_rect.collidepoint(mouse_pos):
                        if password_list:
                            password_list.pop()

                    elif done_rect.collidepoint(mouse_pos) and len(password_list) > 3:
                        if selected_drawer == 1:
                            drawer1["password"] = password
                        elif selected_drawer == 2:
                            drawer2["password"] = password
                        elif selected_drawer == 3:
                            drawer3["password"] = password
                        password_list = []
                        phase = "main"
                
                elif phase == "main":
                    if setting_rect.collidepoint(mouse_pos):
                        phase = "drawer"
                    elif run_rect.collidepoint(mouse_pos):
                        phase = "check"
                    elif drawer_rect.collidepoint(mouse_pos):
                        phase = "load"

                elif phase == "drawer":
                    if drawer1_rect.collidepoint(mouse_pos):
                        selected_drawer = 1
                        phase = "destination"
                    elif drawer2_rect.collidepoint(mouse_pos):
                        selected_drawer = 2
                        phase = "destination"
                    elif drawer3_rect.collidepoint(mouse_pos):
                        selected_drawer = 3
                        phase = "destination"

                elif phase == "check":
                    if yes_rect.collidepoint(mouse_pos):
                        phase = "driving"
                        msg = Int16MultiArray()
                        if drawer1["destination"]:
                            destination_list.append(destination_converter(drawer1["destination"]))
                        if drawer2["destination"]:
                            destination_list.append(destination_converter(drawer2["destination"]))
                        if drawer3["destination"]:
                            destination_list.append(destination_converter(drawer3["destination"]))
                        msg.data = destination_list
                        pub.publish(msg)
                    elif no_rect.collidepoint(mouse_pos):
                        phase = "main"

                elif phase == "arrive":
                    if num1_rect.collidepoint(mouse_pos):
                        password_list.append(1)
                    elif num2_rect.collidepoint(mouse_pos):
                        password_list.append(2)
                    elif num3_rect.collidepoint(mouse_pos):
                        password_list.append(3)
                    elif num4_rect.collidepoint(mouse_pos):
                        password_list.append(4)
                    elif num5_rect.collidepoint(mouse_pos):
                        password_list.append(5)
                    elif num6_rect.collidepoint(mouse_pos):
                        password_list.append(6)
                    elif num7_rect.collidepoint(mouse_pos):
                        password_list.append(7)
                    elif num8_rect.collidepoint(mouse_pos):
                        password_list.append(8)
                    elif num9_rect.collidepoint(mouse_pos):
                        password_list.append(9)
                    elif num0_rect.collidepoint(mouse_pos):
                        password_list.append(0)
                    elif cancel_rect.collidepoint(mouse_pos):
                        if password_list:
                            password_list.pop()

                    elif done_rect.collidepoint(mouse_pos) and len(password_list) > 3:
                        if drawer1["password"] == password:
                            wrong_flag = False
                            selected_drawer = 1
                            try: 
                                destination_list.remove(destination_converter(drawer1["destination"]))
                                phase = "open"
                                # open(selected_drawer)  
                            except:
                                print("wrong")
                                wrong_flag = True
                        elif drawer2["password"] == password:
                            wrong_flag = False
                            selected_drawer = 2
                            try:
                                destination_list.remove(destination_converter(drawer2["destination"]))
                                phase = "open"
                                # open(selected_drawer)
                            except:
                                print("wrong")
                                wrong_flag = True
                        elif drawer3["password"] == password:
                            wrong_flag = False
                            selected_drawer = 3
                            try:
                                destination_list.remove(destination_converter(drawer3["destination"]))
                                phase = "open"
                                # open(selected_drawer)
                            except:
                                print("wrong")
                                wrong_flag = True
                        else:
                            print("wrong")
                            wrong_flag = True
                        password_list = []
                        
                elif phase == "load":
                    if drawer1_rect.collidepoint(mouse_pos):
                        selected_drawer = 1
                        phase = "open"
                        # open(selected_drawer)
                    elif drawer2_rect.collidepoint(mouse_pos):
                        selected_drawer = 2
                        phase = "open"
                        # open(selected_drawer)
                    elif drawer3_rect.collidepoint(mouse_pos):
                        selected_drawer = 3
                        phase = "open"
                        # open(selected_drawer)

                elif phase == "open":
                    if open_rect.collidepoint(mouse_pos):
                        if driving_flag:
                            msg = Bool()
                            msg.data = True
                            go_pub.publish(msg)
                            phase = "driving"
                        else:
                            phase = "main"
                        close(selected_drawer)
                        pygame.display.flip()

                elif phase == "driving":
                    driving_flag = True
                    if destination_list:
                        phase = "arrive"
                    else:
                        phase = "main"
                        drawer1 = {"destination" : "", "password" : ""}
                        drawer2 = {"destination" : "", "password" : ""}
                        drawer3 = {"destination" : "", "password" : ""}
                        driving_flag = False


        screen.fill(back_color)  # 배경색
        if phase == "destination":
            draw_destination()
            write(title_rect, "set destination")

        elif phase == "password":
            write(title_rect, "set password")
            draw_password()
            password = ''.join(str(s) for s in password_list)
            write(password_rect, password, size= 50)

        elif phase == "driving":
            write(title_rect, "delivering ~")

        elif phase == "main":
            draw_main()
            write(title_rect, "Hello")

        elif phase == "drawer":
            draw_drawer()
            write(title_rect, "choose drawer")

        elif phase == "check":
            draw_check()
            write(title_rect, "Are they right?")

        elif phase == "arrive":
            draw_password()
            if wrong_flag == False:
                write(title_rect, "Enter your password")
            else:
                write(title_rect, "Wrong password", color=(255,0,0), size= 70)
            password = ''.join(str(s) for s in password_list)
            write(password_rect, password, size= 50)

        elif phase == "load":
            draw_drawer()
            write(title_rect, "choose drawer to open")

        elif phase == "open":
            global open_count, open_flag
            write(title_rect, "drawer is opening")
            draw_button(open_rect, "Done")
            pygame.display.flip()
            
            if open_flag == False:
                open(selected_drawer)
                open_flag = True

            open_count += 1
            

        pygame.display.flip()

if __name__ == "__main__":
    main()
