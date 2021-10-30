import math


class Map:
    def __init__(self, drawmap):
        if drawmap:
            import pygame as pg
        self.KnownObjects = []

    def AddKnownObject(self, name, x, y, z):
        KnownObject = [name, x, y, z]
        self.KnownObjects.append(KnownObject)
