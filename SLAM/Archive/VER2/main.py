import pygame
import random
import time
import numpy as np
from simulation import *
from network import Network
import os

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (5,25)

pygame.init()

window_dimensions = (900,600)
window = pygame.display.set_mode(window_dimensions)
pygame.display.set_caption("SIMULATION VER2")

objectNumber = 0

GRAY = (110,110,110)
YELLOW = (255,255,0)
BLUE = (0,0,255)

def readData(str, quant):
	try:
		str = str.split(",")
		lst =[]
		for i in range(0,quant):
			lst.append(float(str[i]))
		return lst
	except:
		pass

def genData(tup):
	return str(tup[0]) + "," + str(tup[1]) + "," + str(tup[2])  + "," + str(tup[3]) + "," + str(tup[4]) # + "," + str(tup[5])+ "," + str(tup[6])

def jumble(value, mag):
	value_1 = np.random.normal(value,mag,None)
	return value_1

def main():
	run = True
	yel_cone = True
	N = Network()
	initState = readData(N.getState(), 5)
	c1 = carSim(initState[0],initState[1],initState[2],initState[3],initState[4])

	yellows = []
	blues = []

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

		window.fill(GRAY)

		seen_yellows = []
		seen_blues = []

		a1 = jumble(c1.angle,0.2)
		a2 = jumble(c1.angle,0.2)

		speed1 = jumble(np.sqrt((c1.velX**2) + (c1.velY**2)),0.2)
		speed2 = jumble(np.sqrt((c1.velX**2) + (c1.velY**2)),0.2)

		click = pygame.mouse.get_pressed()
		if click[0] == 1:
			pos = pygame.mouse.get_pos()
			if yel_cone == True:
				sub_lst = [cone(pos[0],pos[1],YELLOW), False]
				yellows.append(sub_lst)
				yel_cone = False
				time.sleep(0.3)
			else:
				sub_lst = [cone(pos[0],pos[1],BLUE), False]
				blues.append(sub_lst)
				yel_cone = True
				time.sleep(0.3)

		for i in yellows:
			i[0].place(window)
			bool = search(c1,i[0],250,70)
			if bool[0] == True:
				i[1] = True
				seen_yellows.append([bool[1],bool[2]])
			else:
				i[1] = False

		for j in blues:
			j[0].place(window)
			bool = search(c1,j[0],250,70)
			if bool[0] == True:
				j[1] = True
			else:
				j[1] = False

		#seen_yellows = []
		for i in yellows:
			if i[1] == True:
				bool = search(c1,i[0],200,50)
				val = (bool[1],bool[2])
				seen_yellows.append(val)
			else:
				pass

		#seen_blues = []
		for j in blues:
			if j[1] == True:
				bool = search(c1,j[0],200,50)
				val = (bool[1],bool[2])
				seen_blues.append(val)
			else:
				pass


		lst_num = random.randint(0,1)
		try:
			if (lst_num == 0):
				cone_num = random.randint(0,len(seen_yellows))
				R = seen_yellows[cone_num][0]
				THETA = seen_yellows[cone_num][1]
				COL = 0
			else:
				cone_num = random.randint(0,len(seen_blues))
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

		print(R,THETA)

		c1State = readData(N.send(genData((COL,speed2,a1,R,THETA))), 5)

		c1.drive()
		c1.update()
		updateWindow(window, GRAY, GRAY, c1, 0,0)

		pygame.display.update()

main()
