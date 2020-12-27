import pygame
import numpy as np
from simulation import *
from network import Network
import os

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (920,25)

pygame.init()

window_dimensions = (900,600)
window = pygame.display.set_mode(window_dimensions)
pygame.display.set_caption("RECONSTRUCTION VER2")

objectNumber = 1

BLACK = (0,0,0)
WHITE = (255,255,255)
WHITE2 = (120,120,100)
YELLOW = (255,255,0)
COL2 = (230,0,0)
BLUE = (0,0,255)

def readData(str,quant):
	try:
		str = str.split(",")
		lst = []
		for i in range(0,quant):
			lst.append(float(str[i]))
		return lst
	except:
		pass

def genData(tup):
	return str(tup[0]) + "," + str(tup[1]) + "," + str(tup[2]) + "," + str(tup[3]) + "," + str(tup[4])

def average(a,b):
	return (a+b)/2

def main2():
	run = True
	N = Network()
	initState = readData(N.getState(), 5)
	c2 = carSim(initState[0],initState[1],initState[2],initState[3],initState[4])
	clock = pygame.time.Clock()

	yellow_cones = []
	blue_cones = []
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
		display_count+=1
		update_count+=1

		if update_count > 25:
			yellow_cones = []
			blue_cones = []
			update_count = 0

		window.fill(BLACK)

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False
				pygame.quit()


		c2State = readData(N.send(genData((c2.x,c2.y,c2.velX,c2.velY,c2.angle))), 5)

		speed = c2State[1]
		c2.velX = speed*np.cos(np.radians(c2State[2]))
		c2.velY = -speed*np.sin(np.radians(c2State[2]))
		c2.angle = c2State[2]

		COL = c2State[0]
		R = c2State[3]
		THETA = c2State[4]
		print(R,THETA)

		if (R != 9999) and (THETA != 9999):
			X = int(c2.x + R*np.cos(np.radians(c2.angle%360+THETA)))
			Y = int(c2.y - R*np.sin(np.radians(c2.angle%360+THETA)))
			try:
				yellow_bool = checkNew(yellow_cones,[X,Y],15)
				blue_bool = checkNew(blue_cones,[X,Y],15)
			except:
				pass
			if (COL == 0) and (yellow_bool == True):
				yellow_cones.append([X,Y])
			elif COL == 1 and (blue_bool == True):
				blue_cones.append([X,Y])
			else:
				pass

		if display_count > 6:
			display_angle = c2.angle%360
			display_speed = speed
			display_count = 0
			yel_num = len(yellow_cones)
			blu_num = len(blue_cones)

		c2.update()
		c2.drive()
		updateWindow(window, BLACK, WHITE2,c2, window_dimensions[0], window_dimensions[1])


		for i in yellow_cones:
			ylx = i[0]
			yly = i[1]
			pygame.draw.circle(window,YELLOW, (ylx,yly), 10)
			pygame.draw.line(window,YELLOW,(c2.x,c2.y), (ylx,yly), 1)

		for j in blue_cones:
			ylx = j[0]
			yly = j[1]
			pygame.draw.circle(window,BLUE, (ylx,yly), 10)
			pygame.draw.line(window,BLUE,(c2.x,c2.y), (ylx,yly), 1)

		display_data("Angle: ", round(display_angle,2), 20,20, FONT,WHITE,window)
		display_data("Speed: ", round(display_speed,2), 200, 20, FONT,WHITE,window)
		display_data("Yellow Cones: ", yel_num, 20, 550, FONT,YELLOW,window)
		display_data("Blue Cones: ", blu_num, 250, 550, FONT,BLUE,window)



		pygame.display.update()

main2()

