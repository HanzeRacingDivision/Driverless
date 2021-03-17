from simulation import *
from network import Network
from host import readData, genData

import os


def main2():
    run = True
    print(f"Connecting to network...")
    N = Network("receiver")
    initState = readData(N.getState())
    print("Connection successful.")
    c2 = carSim(initState[0], initState[1], initState[2], initState[3], initState[4])
    clock = pygame.time.Clock()

    cones = {"yellow": [], "blue": []}
    update_count = 0

    X = None
    Y = None

    FONT = pygame.font.Font('freesansbold.ttf', 25)
    display_count = 0
    display_angle = 0
    display_speed = 0
    yel_num = 0
    blu_num = 0

    while run:

        clock.tick(100)
        display_count += 1
        # update_count += 1

        # if update_count > 25:
        #     cones["yellow"] = []
        #     cones["blue"] = []
        #     update_count = 0

        window.fill(colors["BLACK"])

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()

        c2State = readData(N.send(genData((c2.x, c2.y, c2.velX, c2.velY, c2.angle))))

        speed = c2State[1]
        c2.velX = speed * np.cos(np.radians(c2State[2]))
        c2.velY = -speed * np.sin(np.radians(c2State[2]))
        c2.angle = c2State[2]

        COL = c2State[0]
        R = c2State[3]
        THETA = c2State[4]
        # print(R, THETA)

        if (R != 9999) and (THETA != 9999):
            X = int(c2.x + R * np.cos(np.radians(c2.angle % 360 + THETA)))
            Y = int(c2.y - R * np.sin(np.radians(c2.angle % 360 + THETA)))
            try:
                yellow_bool = checkNew(cones["yellow"], [X, Y], 15)
                blue_bool = checkNew(cones["blue"], [X, Y], 15)
            except:
                pass

            if not COL and yellow_bool:
                cones["yellow"].append([X, Y])
            elif COL and blue_bool:
                cones["blue"].append([X, Y])
            else:
                pass

        if display_count > 6:
            display_angle = c2.angle % 360
            display_speed = speed
            display_count = 0
            yel_num = len(cones["yellow"])
            blu_num = len(cones["blue"])

        c2.update()
        c2.drive()
        updateWindow(window, colors["BLACK"], colors["WHITE2"], c2, window_dimensions[0], window_dimensions[1])

        for cone_type, cones_list in cones.items():
            for cone in cones_list:
                ylx = cone[0]
                yly = cone[1]
                if cone_type == "yellow":
                    color = colors["YELLOW"]
                elif cone_type == "blue":
                    color = colors["BLUE"]
                pygame.draw.circle(window, color, (ylx, yly), 10)
                pygame.draw.line(window, color, (c2.x, c2.y), (ylx, yly), 1)

        display_data("Angle: ", round(display_angle, 2), 20, 20, FONT, colors["WHITE"], window)
        display_data("Speed: ", round(display_speed, 2), 200, 20, FONT, colors["WHITE"], window)
        display_data("Yellow Cones: ", yel_num, 20, 550, FONT, colors["YELLOW"], window)
        display_data("Blue Cones: ", blu_num, 250, 550, FONT, colors["BLUE"], window)

        pygame.display.update()


if __name__ == '__main__':
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (920, 25)

    pygame.init()

    window_dimensions = (900, 600)
    window = pygame.display.set_mode(window_dimensions)
    pygame.display.set_caption("RECONSTRUCTION VER2")

    objectNumber = 1

    colors = {
        "BLACK": (0, 0, 0),
        "WHITE": (255, 255, 255),
        "WHITE2": (120, 120, 100),
        "YELLOW": (255, 255, 0),
        "COL2": (230, 0, 0),
        "BLUE": (0, 0, 255)
    }

    main2()
