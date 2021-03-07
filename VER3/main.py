import random
import time
import os
from simulation import *
from network import Network
from host import readData, genData


def jumble(mean, std):
    value_1 = np.random.normal(mean, std, None)
    return value_1


def main():
    run = True
    N = Network("transmitter")
    initState = readData(N.getState())
    c1 = carSim(initState[0], initState[1], initState[2], initState[3], initState[4])

    cones = {"yellow": [], "blue": []}

    R = 9999
    THETA = 9999
    COL = 9999

    clock = pygame.time.Clock()

    while run:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()

        clock.tick(100)

        window.fill(colors["GRAY"])

        seen_yellows = []
        seen_blues = []

        a1 = jumble(c1.angle, 0.2)
        # a2 = jumble(c1.angle, 0.2)
        #
        # speed1 = jumble(np.sqrt((c1.velX ** 2) + (c1.velY ** 2)), 0.2)
        speed2 = jumble(np.sqrt((c1.velX ** 2) + (c1.velY ** 2)), 0.2)

        click = pygame.mouse.get_pressed()
        pos = pygame.mouse.get_pos()
        if click[0]:
            sub_lst = [Cone(pos[0], pos[1], colors["YELLOW"]), False]
            cones["yellow"].append(sub_lst)
        elif click[-1]:
            sub_lst = [Cone(pos[0], pos[1], colors["BLUE"]), False]
            cones["blue"].append(sub_lst)
        if any(click):
            time.sleep(0.2)

        for cone_type, cones_list in cones.items():
            for cone in cones_list:
                cone[0].place(window)
                bool = search(c1, cone[0], 250, 70)
                if bool[0]:
                    cone[1] = True
                    if cone_type == "yellow":  # yellows
                        seen_yellows.append([bool[1], bool[2]])
                else:
                    cone[1] = False

        for cone_type, cones_list in cones.items():
            for cone in cones_list:
                if cone[1]:
                    bool = search(c1, cone[0], 200, 50)
                    val = (bool[1], bool[2])
                    if cone_type == "yellow":
                        seen_yellows.append(val)
                    else:
                        seen_blues.append(val)

        lst_num = random.randint(0, 1)
        try:
            if lst_num == 0:
                cone_num = random.randint(0, len(seen_yellows))
                R = seen_yellows[cone_num][0]
                THETA = seen_yellows[cone_num][1]
                COL = 0
            else:
                cone_num = random.randint(0, len(seen_blues))
                R = seen_blues[cone_num][0]
                THETA = seen_blues[cone_num][1]
                COL = 1
        except:
            R = 9999
            THETA = 9999
            COL = 9999

        if R == 0:
            R = 9999
            THETA = 9999

        # print(R, THETA)

        c1State = readData(N.send(genData((COL, speed2, a1, R, THETA))))

        c1.drive()
        c1.update()
        updateWindow(window, colors["GRAY"], colors["GRAY"], c1, 0, 0)

        pygame.display.update()


if __name__ == '__main__':
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (5, 25)

    pygame.init()

    window_dimensions = (900, 600)
    window = pygame.display.set_mode(window_dimensions)
    pygame.display.set_caption("SIMULATION VER2")

    objectNumber = 0

    colors = {
        "GRAY": (110, 110, 110),
        "YELLOW": (255, 255, 0),
        "BLUE": (0, 0, 255)
    }

    main()
