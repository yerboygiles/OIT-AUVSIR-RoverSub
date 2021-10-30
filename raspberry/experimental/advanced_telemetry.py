import pygame
import math
from pygame.locals import *
import sys
from OpenGL.GL import *
from OpenGL.GLU import *

verticies = (
    (0, 3, 1),
    (2, -2, 1),
    (-2, -2, 1),
    (0, -1, 1),
    (0, 3, 0),
    (2, -2, 0),
    (-2, -2, 0),
    (-0, -1, 0)
)
edges = (
    (0, 1),
    (1, 3),
    (3, 2),
    (2, 0),

    (4, 5),
    (5, 7),
    (7, 6),
    (6, 4),

    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7)
)
surfaces = (
    (0, 1, 3, 2),
    (4, 5, 7, 6),

    (0, 2, 6, 4),
    (2, 3, 7, 6),

    (3, 7, 5, 1),
    (5, 1, 0, 4)
)
colors = (
    (1, 0, 0),
    (0, 1, 0),
    (0, 0, 1),
    (0, 1, 0),
    (1, 1, 1),
    (0, 1, 1),
    (1, 0, 0),
    (0, 1, 0),
    (0, 0, 1),
    (1, 0, 0),
    (1, 1, 1),
    (0, 1, 1),
)


class Telemetry:
    def __init__(self):
        pygame.init()
        display = (800, 600)
        pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

        gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)

        glTranslatef(0, 0, -20)

        object_passed = False
        # glRotatef(25, 2, 1, 0)

    def NavMarker(self):
        glBegin(GL_QUADS)
        for surface in surfaces:
            x = 0
            for vertex in surface:
                x += 1
                glColor3fv(colors[x])
                glVertex3fv(verticies[vertex])
        glEnd()
        glBegin(GL_LINES)
        for edge in edges:
            for vertex in edge:
                glVertex3fv(verticies[vertex])
        glEnd()

    def update(self, yaw):
        glRotatef(yaw, 0, 0, -10)
        pitch = 0
        roll = 0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.NavMarker()
        pygame.display.flip()
        glRotatef(-yaw, 0, 0, -10)
        # glRotatef(-roll, 1, 0, 0)
        # glRotatef(-yaw, 0, 0, 1)
        # glRotatef(-pitch, 0, 1, 0)


clock = pygame.time.Clock()
Telemetry()
simExit = False
while not simExit:
    clock.tick(60)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            simExit = True
            pygame.quit()
            sys.exit()
