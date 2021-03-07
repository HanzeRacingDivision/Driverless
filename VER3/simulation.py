import pygame
import numpy as np


class carSim:

    def __init__(self, x, y, velX, velY, angle):
        # carSimImg = pygame.image.load('big-car.png')
        self.x = x
        self.y = y
        self.velX = velX
        self.velY = velY
        self.angle = angle
        self.vel = 0.3
        self.angVel = 1.5
        self.state = (self.x, self.y, self.velX, self.velY, self.angle)

    def manifest(self, window):
        carSimImg = pygame.image.load('racing-car.png')
        sprite = pygame.transform.rotate(carSimImg, self.angle)
        width = int(sprite.get_width())
        height = int(sprite.get_height())
        window.blit(sprite, (self.x - width / 2, self.y - height / 2))

    def drive(self):

        decay = 0.05
        cutoff = 0.001
        vel_limiter = 0.6
        keys = pygame.key.get_pressed()

        if keys[pygame.K_LEFT]:
            self.angle += self.angVel
        if keys[pygame.K_RIGHT]:
            self.angle -= self.angVel
        if keys[pygame.K_UP]:
            self.velX += self.vel * (np.cos(self.angle * 3.14159 / 180)) * vel_limiter
            self.velY += self.vel * (-np.sin(self.angle * 3.14159 / 180)) * vel_limiter
        if keys[pygame.K_DOWN]:
            self.velX -= self.vel * (np.cos(self.angle * 3.14159 / 180)) * vel_limiter
            self.velY -= self.vel * (-np.sin(self.angle * 3.14159 / 180)) * vel_limiter

        if abs(self.velX) > cutoff:
            self.velX -= decay * self.velX
        else:
            self.velX = 0
        if abs(self.velY) > cutoff:
            self.velY -= decay * self.velY
        else:
            self.velY = 0

        self.x += self.velX
        self.y += self.velY

    def update(self):
        self.state = (self.x, self.y, self.velX, self.velY, self.angle)


class Cone():
    def __init__(self, x, y, colour):
        self.x = x
        self.y = y
        self.colour = colour

    def place(self, window):
        pygame.draw.circle(window, self.colour, (self.x, self.y), 10)


def getAngle(v1, v2):
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    cross_product = np.cross(v1, v2)
    sign = cross_product / abs(cross_product)
    v1_norm = np.sqrt(v1[0] ** 2 + v1[1] ** 2)
    v2_norm = np.sqrt(v2[0] ** 2 + v2[1] ** 2)
    angle = np.arccos(dot_product / (v1_norm * v2_norm))
    return sign * (np.degrees(angle) % 360)


def search(car, cone, max_R, angle_bound):
    v1_x = cone.x - car.x
    v1_y = cone.y - car.y
    v1 = [v1_x, v1_y]

    v2_x = 30 * np.cos(np.radians(car.angle % 360))
    v2_y = -30 * np.sin(np.radians(car.angle % 360))
    v2 = [v2_x, v2_y]
    R = np.sqrt(v1_x ** 2 + v1_y ** 2)
    theta = getAngle(v1, v2)
    # print(theta)

    if R < max_R:
        if -angle_bound < theta < angle_bound:
            return True, R, theta
        else:
            return False, 0, 0
    else:
        return False, 0, 0


def checkNew(lst, value, bound):
    bool = True
    for element in lst:
        print(abs(element[0] - value[0]), bound)
        if (abs(element[0] - value[0]) < bound) and (abs(element[1] - value[1]) < bound):
            print(abs(element[0] - value[0]), bound)
            bool = False
            break

    return bool


def display_data(data_str, data_val, x, y, font, colour, window):
    display = font.render(data_str + str(data_val), True, colour)
    window.blit(display, (x, y))


def genGrid(XD, YD, gap, window, colour):
    for i in range(0, XD, gap):
        pygame.draw.line(window, colour, (i, 0), (i, YD), 1)
    for j in range(0, YD, gap):
        pygame.draw.line(window, colour, (0, j), (XD, j), 1)


def updateWindow(window, colour1, colour2, object, XD, YD):
    genGrid(XD, YD, 15, window, colour2)
    object.manifest(window)
