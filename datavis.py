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

width, height = 640, 480
center_y, center_x = height / 2, width / 2
screen = pygame.display.set_mode((width, height))
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
radius_draw_mult = 300
rect_width = 16
line_width = 1
triangle_width = 10
triangle_height = 20
control_force_indicator = 0
control_force_indicator_multiplier = pi/6
font = pygame.font.SysFont(None, 24)

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
    global position, rcs_direction, setpoint, deadzone, altitude, control_force_indicator, packet, state
    position = row[17]
    rcs_direction = -1 if row[30] == 1 else 1 if row[31] == 1 else 0
    setpoint = row[24]
    deadzone = row[28]
    altitude = row[5]
    control_force_indicator = row[27]
    packet = row[2]
    state = flight_states[row[3]]

def drawRCS():
    rect = [
        [radius * radius_draw_mult, rect_width / 2],
        [radius * radius_draw_mult, -rect_width / 2],
        [-radius * radius_draw_mult, -rect_width / 2],
        [-radius * radius_draw_mult, rect_width / 2],
        [-rect_width / 2, rect_width / 2],
        [-rect_width / 2, rect_width * 1.5],
        [rect_width / 2, rect_width * 1.5],
        [rect_width / 2, rect_width / 2]
    ]
    for point in rect:
        x = point[0]
        y = point[1]
        point[0] = x * cos(position) - y * sin(position)
        point[1] = y * cos(position) + x * sin(position)
        point[0] += center_x
        point[1] += center_y
    pygame.draw.lines(screen, WHITE, True, rect, line_width)

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
    if control_force_indicator != 0:
        pygame.draw.circle(screen, WHITE, (center_x, height / 12), 4, 0)
        if control_force_indicator > 0:
            pygame.draw.arc(screen, WHITE, pygame.Rect(center_x - 5 * height/ 12, height / 12, 10 * height / 12, 10 * height / 12),
                        pi/2, pi/2 + control_force_indicator * control_force_indicator_multiplier, 2)
        else:
            pygame.draw.arc(screen, WHITE, pygame.Rect(center_x - 5 * height/ 12, height / 12, 10 * height / 12, 10 * height / 12),
                        pi/2 + control_force_indicator * control_force_indicator_multiplier, pi/2, 2)
    pygame.draw.line(screen, YELLOW,
        (100 * cos(position - pi / 2) + center_x, 100 * sin(position - pi / 2) + center_y),
        (150 * cos(position - pi / 2) + center_x, 150 * sin(position - pi / 2) + center_y)
    )
    pygame.draw.line(screen, RED,
        (100 * cos(position - pi / 2 - deadzone) + center_x, 100 * sin(position - pi / 2 - deadzone) + center_y),
        (150 * cos(position - pi / 2 - deadzone) + center_x, 150 * sin(position - pi / 2 - deadzone) + center_y)
    )
    pygame.draw.line(screen, RED,
        (100 * cos(position - pi / 2 + deadzone) + center_x, 100 * sin(position - pi / 2 + deadzone) + center_y),
        (150 * cos(position - pi / 2 + deadzone) + center_x, 150 * sin(position - pi / 2 + deadzone) + center_y)
    )
    pygame.draw.line(screen, GREEN,
        (50 * cos(setpoint - pi / 2) + center_x, 50 * sin(setpoint - pi / 2) + center_y),
        (200 * cos(setpoint - pi / 2) + center_x, 200 * sin(setpoint - pi / 2) + center_y)
    )

    time_text = font.render(str(round(time/1000,3)), True, WHITE)
    screen.blit(time_text, (30,30))
    alt_text = font.render(str(round(altitude,1)), True, WHITE)
    screen.blit(alt_text, (30,45))
    packet_text = font.render(str(int(packet)), True, WHITE)
    screen.blit(packet_text, (30,15))
    state_text = font.render(state, True, WHITE)
    screen.blit(state_text, (30,60))


while True:
    screen.fill(BLACK)

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == KEYDOWN:
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
