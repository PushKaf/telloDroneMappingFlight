from djitellopy import Tello
import pygame
from time import sleep


# ####NOTE####
# This is just a utility module used for my other tello programs.
# ############


_width = 169
_height = 142

def init():
    pygame.init() #initalize pygame
    win = pygame.display.set_mode((_width, _height)) #make a window and set its dims

def getKey(key):
    answer = False

    for events in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(key))

    if keyInput[myKey]:
        answer = True   

    pygame.display.update()

    return answer

def main():
    print(getKey("a"))


if __name__ == "__main__":
    init()
    while True:
        main()