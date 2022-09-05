import sys
import pygame
from pygame.locals import *
from math import sin, cos, radians, degrees, pi, atan2

fps = 60

radius = .5
pressure = 344738
mass = 1.14
rotational_inertia = mass * 0.015484
nozzle_area = .000001


max_frequency = 10
cycles_in_current_state = 0
current_state = 0
recent_cycles = [0 for x in range(int(fps/3))]
output_control = 0


altitude = 30 #kilometers
if altitude <= 11:
    t = 15.04 - .00649 * altitude * 1000
    altitude_pressure = 101.29 * ((t + 273.1)/288.08) ** 5.256
if 11 < altitude <= 25:
    t = -56.46
    altitude_pressure = 22.65 * 2.71 ** (1.73 - .000157 * altitude * 1000)
if altitude > 25:
    t = -131.21 + .00299 * altitude * 1000
    altitude_pressure = 2.488 * ((t + 273.1)/ 216.6) ** -11.388
altitude_pressure *= 1000

position = 0
velocity = 0
accel = 0

integral_positions = [0,0,0,0,0,0,0,0,0,0,0,0,0]
proportional = .2
integral = .1
derivative = -.7
setpoint = 0
deadzone = .2
deadspeed = .2

rcs_direction = 0

pygame.init()

fpsClock = pygame.time.Clock()

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
control_force_indicator_multiplier = width


def PID_control(pos, vel, target):
    global control_force_indicator
    integral_positions.pop(0)
    integral_positions.append(target - pos)
    if abs(vel) < deadspeed and abs(target - pos) < deadzone:
        desired_force = 0
    else:
        desired_force = (target - pos) * proportional + (vel) * derivative + (sum(integral_positions)) * integral
    control_force_indicator = desired_force
    return desired_force

def PWM_control(rcs_force, desired_force):
    global output_control, cycles_in_current_state
    max_force = .2
    min_force = .05
    if cycles_in_current_state < fps/max_frequency:
        cycles_in_current_state += 1
    elif output_control == 0:
        if desired_force > max_force:
            output_control = 1
            cycles_in_current_state = 0
        elif desired_force < -max_force:
            output_control = -1
            cycles_in_current_state = 0
    elif output_control == 1:
        if desired_force < min_force:
            output_control = 0
            cycles_in_current_state = 0
    elif output_control == -1:
        if desired_force > -min_force:
            output_control = 0
            cycles_in_current_state = 0
    return 0 if pygame.key.get_pressed()[pygame.K_LEFT] and pygame.key.get_pressed()[pygame.K_RIGHT] else -1 if pygame.key.get_pressed()[pygame.K_LEFT] else 1 if pygame.key.get_pressed()[pygame.K_RIGHT] else output_control


def updateRCS():
    global accel, velocity, position, rcs_direction, setpoint
    if pygame.mouse.get_pressed()[0]:
        setpoint = atan2(pygame.mouse.get_pos()[0]-center_x, -(pygame.mouse.get_pos()[1]-center_y))
    while position < setpoint-pi:
        position += 2 * pi
    while position > setpoint+pi:
        position -= 2 * pi

    rcs_direction = 0
    rcs_force = (pressure * nozzle_area) - (altitude_pressure * nozzle_area)
    applied_force = 0;

    desired_force = PID_control(position, velocity, setpoint)
    rcs_direction = PWM_control(rcs_force, desired_force)
    applied_force = rcs_force * rcs_direction
    
    accel = (applied_force * radius / rotational_inertia)
    velocity += accel / fps
    position += velocity / fps


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
    pygame.draw.lines(screen, WHITE if not pygame.key.get_pressed()[pygame.K_LEFT] and not pygame.key.get_pressed()[pygame.K_RIGHT] else RED, True, rect, line_width)

    if rcs_direction != 0 and not pygame.key.get_pressed()[pygame.K_LEFT] and not pygame.key.get_pressed()[pygame.K_RIGHT]:
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
    # if control_force_indicator != 0:
    #     pygame.draw.line(screen, WHITE, (center_x, height / 12), (center_x + control_force_indicator * control_force_indicator_multiplier, height / 12))
    if not pygame.key.get_pressed()[pygame.K_LEFT] and not pygame.key.get_pressed()[pygame.K_RIGHT]:
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


while True:
    screen.fill(BLACK)

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

    
    if pygame.key.get_pressed()[pygame.K_UP]:
        position = 0
        velocity = 0
        accel = 0
        setpoint = 0

    updateRCS()
    drawRCS()

    pygame.display.flip()
    fpsClock.tick(fps)
