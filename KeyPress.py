import pygame


def init():
    pygame.init()
    win = pygame.display.set_mode((400, 400))


def get_key(key_name) -> bool:
    ans = False
    for _ in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, "K_{}".format(key_name))
    if keyInput[myKey]:
        ans = True

    pygame.display.update()
    return ans
