import sys
import pygame
from pygame.locals import *
from math import sin, cos, radians, degrees, pi, atan2
import csv

fps = 60

radius = .5

position = 0
velocity = 0
accel = 0
rcs_direction = 0
setpoint = 0
deadzone = 0
deadspeed = 0
altitude = 0
packet = 0
state = "LAUNCH"
flight_states = {0: "TEST", 1: "LAUNCH", 2: "ASCENT", 3: "STABILIZATION", 4: "DESCENT", 5: "LANDING", 6: "LANDED"}
pygame.init()
time = 0
fpsClock = pygame.time.Clock()
auto_forward = False
velocity = 0

width, height = 1280, 720
center_y, center_x = height / 2, width / 2
screen = pygame.display.set_mode((width, height))
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
radius_draw_mult = 301
rect_width = 40
line_width = 3
triangle_width = 15
triangle_height = 20
control_force_indicator = 0
control_force_indicator_multiplier = pi/6
velocity_indicator_multiplier = pi/3
font = pygame.font.SysFont(None, 24)
bigfont = pygame.font.SysFont(None, 48)
sunflower_image = pygame.image.load("sunflowertop.png")
rotation_multiplier = - 180 / pi

with open("LOG00199.txt", "r") as flight_data_file:
    reader = csv.reader(flight_data_file, delimiter=",")
    flight_data = []
    for row in reader:
        flight_data.append(row)

for id_row, row in enumerate(flight_data):
    for id_column, column in enumerate(row):
        try:
            flight_data[id_row][id_column] = float(flight_data[id_row][id_column])
        except:
            pass

flight_data.pop(-1)
total_packets = flight_data[-1][2]-1

def getClosestPacket():
    global time
    for id_row, row in enumerate(flight_data):
        if id_row == 0:
            continue
        if row[1] >= time:
            return flight_data[id_row-1]
    return flight_data[-1]

def updateTime():
    global time, auto_forward
    if pygame.key.get_pressed()[pygame.K_LCTRL]:
        if pygame.key.get_pressed()[pygame.K_LEFT]:
            time -= int(300000/fps)
            auto_forward = False
        if pygame.key.get_pressed()[pygame.K_RIGHT]:
            time += int(300000/fps)
            auto_forward = False
    elif pygame.key.get_pressed()[pygame.K_LSHIFT]:
        if pygame.key.get_pressed()[pygame.K_LEFT]:
            time -= int(60000/fps)
            auto_forward = False
        if pygame.key.get_pressed()[pygame.K_RIGHT]:
            time += int(60000/fps)
            auto_forward = False
    else:
        if pygame.key.get_pressed()[pygame.K_LEFT]:
            time -= int(1000/fps)
            auto_forward = False
        if pygame.key.get_pressed()[pygame.K_RIGHT]:
            time += int(1000/fps)
            auto_forward = False
    if time < flight_data[0][1]:
        time = flight_data[0][1]
    elif time > flight_data[-1][1]:
        time = flight_data[-1][1]
        auto_forward = False
    if auto_forward:
        time += int(1000/fps)

def frame_skip(frames):
    global time, auto_forward
    auto_forward = False
    packet = int(getClosestPacket()[2])+1
    try:
        time = flight_data[packet+frames][1]
    except:
        time = flight_data[packet][1]

def processRow(row):
    global position, rcs_direction, setpoint, deadzone, altitude, control_force_indicator, packet, state, velocity
    position = row[17]
    rcs_direction = -1 if row[30] == 1 else 1 if row[31] == 1 else 0
    setpoint = row[24]
    deadzone = row[28]
    altitude = row[5]
    control_force_indicator = row[27]
    packet = row[2]
    state = flight_states[row[3]]
    velocity = row[12]

def drawRCS():
    global time
    rotated_sunflower = pygame.transform.rotozoom(sunflower_image, position * rotation_multiplier, .273)
    screen.blit(rotated_sunflower, (center_x - rotated_sunflower.get_width() / 2, center_y - rotated_sunflower.get_height() / 2))

    if rcs_direction != 0:
        if rcs_direction > 0:
            triangle_one = [
                [radius * radius_draw_mult, -rect_width / 2],
                [radius * radius_draw_mult + triangle_width / 2, -rect_width / 2 - triangle_height],
                [radius * radius_draw_mult - triangle_width / 2, -rect_width / 2 - triangle_height]
            ]
            triangle_two = [
                [-radius * radius_draw_mult, rect_width / 2],
                [-radius * radius_draw_mult - triangle_width / 2, rect_width / 2 + triangle_height],
                [-radius * radius_draw_mult + triangle_width / 2, rect_width / 2 + triangle_height]
            ]
        if rcs_direction < 0:
            triangle_one = [
                [radius * radius_draw_mult, rect_width / 2],
                [radius * radius_draw_mult + triangle_width / 2, rect_width / 2 + triangle_height],
                [radius * radius_draw_mult - triangle_width / 2, rect_width / 2 + triangle_height]
            ]
            triangle_two = [
                [-radius * radius_draw_mult, -rect_width / 2],
                [-radius * radius_draw_mult - triangle_width / 2, -rect_width / 2 - triangle_height],
                [-radius * radius_draw_mult + triangle_width / 2, -rect_width / 2 - triangle_height]
            ]
 
        for point in triangle_one:
            x = point[0]
            y = point[1]
            point[0] = x * cos(position) - y * sin(position)
            point[1] = y * cos(position) + x * sin(position)
            point[0] += center_x
            point[1] += center_y
        for point in triangle_two:
            x = point[0]
            y = point[1]
            point[0] = x * cos(position) - y * sin(position)
            point[1] = y * cos(position) + x * sin(position)
            point[0] += center_x
            point[1] += center_y
        pygame.draw.lines(screen, WHITE, True, triangle_one, line_width)
        pygame.draw.lines(screen, WHITE, True, triangle_two, line_width)
    pygame.draw.circle(screen, GREEN, (center_x, 2 * height / 12 + 1), line_width * 2, 0)
    if velocity > 0:
        pygame.draw.arc(screen, GREEN, pygame.Rect(center_x - 4 * height / 12, 2 * height / 12, 8 * height / 12, 8 * height / 12),
                    pi/2 - velocity * velocity_indicator_multiplier, pi/2, line_width)
    else:
       pygame.draw.arc(screen, GREEN, pygame.Rect(center_x - 4 * height / 12, 2 * height / 12, 8 * height / 12, 8 * height / 12),
                    pi/2, pi/2 - velocity * velocity_indicator_multiplier, line_width)
    pygame.draw.circle(screen, WHITE, (center_x, height / 12 + 1), line_width * 2, 0)
    if control_force_indicator != 0:
        if control_force_indicator > 0:
            pygame.draw.arc(screen, WHITE, pygame.Rect(center_x - 5 * height / 12, height / 12, 10 * height / 12, 10 * height / 12),
                        pi/2, pi/2 + control_force_indicator * control_force_indicator_multiplier, line_width)
        else:
            pygame.draw.arc(screen, WHITE, pygame.Rect(center_x - 5 * height/ 12, height / 12, 10 * height / 12, 10 * height / 12),
                        pi/2 + control_force_indicator * control_force_indicator_multiplier, pi/2, line_width)
    pygame.draw.line(screen, YELLOW,
        (100 * cos(position - pi / 2) + center_x, 100 * sin(position - pi / 2) + center_y),
        (150 * cos(position - pi / 2) + center_x, 150 * sin(position - pi / 2) + center_y),
        line_width)
    pygame.draw.line(screen, RED,
        (100 * cos(position - pi / 2 - deadzone) + center_x, 100 * sin(position - pi / 2 - deadzone) + center_y),
        (150 * cos(position - pi / 2 - deadzone) + center_x, 150 * sin(position - pi / 2 - deadzone) + center_y),
        line_width)
    pygame.draw.line(screen, RED,
        (100 * cos(position - pi / 2 + deadzone) + center_x, 100 * sin(position - pi / 2 + deadzone) + center_y),
        (150 * cos(position - pi / 2 + deadzone) + center_x, 150 * sin(position - pi / 2 + deadzone) + center_y),
        line_width)
    pygame.draw.line(screen, GREEN,
        (75 * cos(setpoint - pi / 2) + center_x, 75 * sin(setpoint - pi / 2) + center_y),
        (175 * cos(setpoint - pi / 2) + center_x, 175 * sin(setpoint - pi / 2) + center_y),
        line_width)

    time_text = bigfont.render("T+0"+str(int(time//3600000))+":"+ ("0"+str(int(time//60000)) if time // 60000 < 10 else str(int(time//60000)))+":"+("0"+str(round((time/1000)%60,3) if time > 10000 else str(round((time/1000)%60,3)))), True, WHITE)
    screen.blit(time_text, (center_x - 130, height - 45))
    alt_text = font.render(str(round(altitude,1)), True, WHITE)
    screen.blit(alt_text, (30,45))
    packet_text = bigfont.render("PACKET "+str(int(packet))+"/"+str(int(total_packets)), True, WHITE)
    screen.blit(packet_text, (15,height - 45))
    state_text = bigfont.render(state, True, WHITE)
    screen.blit(state_text, (width - 280, height - 45))


while True:
    screen.fill(BLACK)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                auto_forward = not auto_forward
            elif event.key == pygame.K_PERIOD:
                frame_skip(1)
            elif event.key == pygame.K_COMMA:
                frame_skip(-1)

    updateTime()
    processRow(getClosestPacket())
    drawRCS()

    pygame.display.flip()
    fpsClock.tick(fps)
