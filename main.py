import pygame as pg
import numpy as np
import scipy as sp
import sys

class PID:
    def __init__(self, set_point, Kp, Ki, Kd, log = False):
        self.set_point = set_point
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.log = log

    def update(self, dt, process_var):
        err = self.set_point - process_var
        self.integral += err * dt
        deriv = err / dt

        u = self.Kp * err + self.Ki * self.integral + self.Kd * deriv

        return u
    
    def new_goal(self, set_point):
        self.integral = 0
        self.set_point = set_point

def render_text_info(screen, curr_x, curr_y, set_point):
    font = pg.font.Font(None, 36)

    # draw the goal position
    text_SP = font.render(f'Set_point = X: {set_point[0]}, Y: {set_point[1]}', True, (255, 255, 255))
    # draw the current position
    text_CP = font.render(f'Current_point = X: {curr_x}, Y: {curr_y}', True, (255, 255, 255))

    text_rect_SP = text_SP.get_rect(center=(text_SP.get_rect().width // 2, text_SP.get_rect().height // 2))
    screen.blit(text_SP, text_rect_SP)

    text_rect_CP = text_CP.get_rect(center=(text_CP.get_rect().width // 2, text_SP.get_rect().height + text_CP.get_rect().height // 2))
    screen.blit(text_CP, text_rect_CP)

def draw_set_point(screen, color, pos):
    pg.draw.circle(screen, color, pos, 10)

def draw_curr_point(screen, color, pos):
    pg.draw.circle(screen, color, pos, 5)

if __name__ == "__main__":
    pg.init()

    screen_size = (1980, 1080)
    screen = pg.display.set_mode(screen_size)
    pg.display.set_caption("PID Follower")

    circle_color = (255, 255, 255)

    set_point = None
    curr_point = None

    pid_x = None
    pid_y = None

    dt = 0
    t = 0

    clock = pg.time.Clock()

    running = True
    while running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False
            if event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
                x, y = pg.mouse.get_pos()
                if pid_x is None and pid_y is None:
                    pid_x = PID(x, 0.8, 1.1, 0.1)
                    pid_y = PID(y, 0.8, 1.1, 0.1)
                    set_point = (x, y)
                    curr_point = (0, 0)
                else:
                    pid_x.new_goal(x)
                    pid_y.new_goal(y)
                    set_point = (x, y)

        screen.fill((0, 0, 0))

        if pid_x is not None and pid_y is not None:
            curr_x = curr_point[0] + pid_x.update(dt, curr_point[0])
            curr_y = curr_point[1] + pid_y.update(dt, curr_point[1])

            curr_point = (curr_x, curr_y)
        
        if set_point is not None and curr_point is not None:
            draw_set_point(screen, circle_color, set_point)
            draw_curr_point(screen, (255, 0, 0), curr_point)

            render_text_info(screen, curr_point[0], curr_point[1], set_point)

        pg.display.flip()

        dt = clock.tick(10) / 1000.0
        t += dt


    pg.quit()
    sys.exit()