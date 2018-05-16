#!/usr/bin/env python
from itertools import cycle
import random
import sys
from laser import Laser
import numpy as np
import pygame.surfarray as surfarray
import matplotlib.pyplot as plt
import pygame
from pygame.locals import *

#rospy
import rospkg
import rospy
from geometry_msgs.msg import Vector3

DEBUG = 0
FPS = 30
SCREENWIDTH  = 432
SCREENHEIGHT = 512

# laser specs
LASERFOV = 90
LASERRES = 9

# scale pixels to meters
SCALING = 0.01
ACCXLIMIT = 3.0
ACCYLIMIT = 35.0
VELLIMIT = 10.0 / (SCALING*FPS)

PIPESPACING  = 192
# amount by which base can maximum shift to left
PIPEGAPSIZE  = 50 # gap between upper and lower part of pipe
BASEY        = SCREENHEIGHT * 0.79
# image, sound and hitmask  dicts
IMAGES, SOUNDS, HITMASKS = {}, {}, {}

rospack = rospkg.RosPack()
PATHTOFLAPPY = rospack.get_path('flappy_main_game')

# list of all possible players (tuple of 3 positions of flap)
PLAYERS_LIST = (
    # red bird
    (
        PATHTOFLAPPY + '/assets/sprites/redbird-upflap.png',
        PATHTOFLAPPY + '/assets/sprites/redbird-midflap.png',
        PATHTOFLAPPY + '/assets/sprites/redbird-downflap.png',
    ),
)

# list of backgrounds
BACKGROUNDS_LIST = (
    PATHTOFLAPPY + '/assets/sprites/background-night.png',
)

# list of pipes
PIPES_LIST = (
    PATHTOFLAPPY + '/assets/sprites/pipe-red.png',
)

playerAccX = 0
playerAccY = 0

try:
    xrange
except NameError:
    xrange = range


def main():
    # init ros node
    rospy.init_node('main_game', anonymous=True)
    #define subscribers
    rospy.Subscriber("flappy_acc", Vector3, controlCallback)
    #define shutdown
    rospy.on_shutdown(shutdownHoook)

    global SCREEN, FPSCLOCK
    pygame.init()
    FPSCLOCK = pygame.time.Clock()
    SCREEN = pygame.display.set_mode((SCREENWIDTH, SCREENHEIGHT))
    pygame.display.set_caption('Flappy Bird')

    # numbers sprites for score display
    IMAGES['numbers'] = (
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/0.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/1.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/2.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/3.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/4.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/5.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/6.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/7.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/8.png').convert_alpha(),
        pygame.image.load(PATHTOFLAPPY + '/assets/sprites/9.png').convert_alpha()
    )

    # game over sprite
    IMAGES['gameover'] = pygame.image.load(PATHTOFLAPPY + '/assets/sprites/gameover.png').convert_alpha()
    # message sprite for welcome screen
    IMAGES['message'] = pygame.image.load(PATHTOFLAPPY + '/assets/sprites/message.png').convert_alpha()
    # base (ground) sprite
    IMAGES['base'] = pygame.image.load(PATHTOFLAPPY + '/assets/sprites/base.png').convert_alpha()

    # sounds
    if 'win' in sys.platform:
        soundExt = '.wav'
    else:
        soundExt = '.ogg'

    SOUNDS['die']    = pygame.mixer.Sound(PATHTOFLAPPY + '/assets/audio/die' + soundExt)
    SOUNDS['hit']    = pygame.mixer.Sound(PATHTOFLAPPY + '/assets/audio/hit' + soundExt)
    SOUNDS['point']  = pygame.mixer.Sound(PATHTOFLAPPY + '/assets/audio/point' + soundExt)
    SOUNDS['swoosh'] = pygame.mixer.Sound(PATHTOFLAPPY + '/assets/audio/swoosh' + soundExt)
    SOUNDS['wing']   = pygame.mixer.Sound(PATHTOFLAPPY + '/assets/audio/wing' + soundExt)

    while True:
        # select random background sprites
        randBg = 0;
        IMAGES['background'] = pygame.image.load(BACKGROUNDS_LIST[randBg]).convert()

        # select random player sprites
        randPlayer = 0;
        IMAGES['player'] = (
            pygame.image.load(PLAYERS_LIST[randPlayer][0]).convert_alpha(),
            pygame.image.load(PLAYERS_LIST[randPlayer][1]).convert_alpha(),
            pygame.image.load(PLAYERS_LIST[randPlayer][2]).convert_alpha(),
        )

        # select random pipe sprites
        pipeindex = 0
        IMAGES['pipe'] = (
            pygame.transform.rotate(
                pygame.image.load(PIPES_LIST[pipeindex]).convert_alpha(), 180),
            pygame.image.load(PIPES_LIST[pipeindex]).convert_alpha(),
        )

        # hismask for pipes
        HITMASKS['pipe'] = (
            getHitmask(IMAGES['pipe'][0]),
            getHitmask(IMAGES['pipe'][1]),
        )

        # hitmask for player
        HITMASKS['player'] = (
            getHitmask(IMAGES['player'][0]),
            getHitmask(IMAGES['player'][1]),
            getHitmask(IMAGES['player'][2]),
        )

        movementInfo = showWelcomeAnimation()
        crashInfo = mainGame(movementInfo)
        showGameOverScreen(crashInfo)


def showWelcomeAnimation():
    """Shows welcome screen animation of flappy bird"""
    # index of player to blit on screen
    playerIndex = 0
    playerIndexGen = cycle([0, 1, 2, 1])
    # iterator used to change playerIndex after every 5th iteration
    loopIter = 0

    playerx = int(SCREENWIDTH*0.13)
    playery = int((SCREENHEIGHT - IMAGES['player'][0].get_height()) / 2)

    messagex = int((SCREENWIDTH - IMAGES['message'].get_width()) / 2)
    messagey = int(SCREENHEIGHT * 0.12)

    basex = 0
    # amount by which base can maximum shift to left
    baseShift = IMAGES['base'].get_width() - IMAGES['background'].get_width()

    # player shm for up-down motion on welcome screen
    playerShmVals = {'val': 0, 'dir': 1}

    while True:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN and (event.key == K_UP):
                # make first flap sound and return values for mainGame
                #SOUNDS['wing'].play()
                return {
                    'playery': playery + playerShmVals['val'],
                    'basex': basex,
                    'playerIndexGen': playerIndexGen,
                }

        # adjust playery, playerIndex, basex
        if (loopIter + 1) % 5 == 0:
            playerIndex = next(playerIndexGen)
        loopIter = (loopIter + 1) % 30
        basex = -((-basex ) % baseShift)
        playerShm(playerShmVals)

        # draw sprites
        SCREEN.blit(IMAGES['background'], (0,0))
        SCREEN.blit(IMAGES['player'][playerIndex],
                    (playerx, playery + playerShmVals['val']))
        SCREEN.blit(IMAGES['message'], (messagex, messagey))
        SCREEN.blit(IMAGES['base'], (basex, BASEY))

        pygame.display.update()
        FPSCLOCK.tick(FPS)


def mainGame(movementInfo):
    global playerAccX
    global playerAccY
    #define publishers
    pub_velocity = rospy.Publisher('flappy_vel', Vector3, queue_size=10)
    # create laser
    laser = Laser(LASERFOV,LASERRES,SCALING)
    score = playerIndex = loopIter = 0
    playerIndexGen = movementInfo['playerIndexGen']
    playerx, playery = int(SCREENWIDTH * 0.13), movementInfo['playery']

    #create timer and counter for timer countdown
    pygame.time.set_timer(pygame.USEREVENT, 1000)
    countdown = 60

    #
    basex = movementInfo['basex']
    baseShift = IMAGES['base'].get_width() - IMAGES['background'].get_width()

    # get 2 new pipes to add to upperPipes lowerPipes list
    newPipe1 = getRandomPipe()
    newPipe2 = getRandomPipe()

    # list of upper pipes
    upperPipes = [
        {'x': SCREENWIDTH + 200, 'y': newPipe1[0]['y']},
        {'x': SCREENWIDTH + 200 + PIPESPACING, 'y': newPipe2[0]['y']},
    ]

    # list of lowerpipe
    lowerPipes = [
        {'x': SCREENWIDTH + 200, 'y': newPipe1[1]['y']},
        {'x': SCREENWIDTH + 200 + PIPESPACING, 'y': newPipe2[1]['y']},
    ]

    # player velocity, max velocity, downward accleration, accleration on flap
    pipeVelX        = 0
    playerVelY      = 0   # player's velocity along Y
    deltaVel        = 0.8
    betweenPipes    = 0

    while True:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN and (event.key == K_UP):
                #if playery > -2 * IMAGES['player'][0].get_height():
                playerVelY -= deltaVel
                #playerAccY -= deltaAcc
            if event.type == KEYDOWN and (event.key == K_DOWN):
                playerVelY += deltaVel
                #playerAccY += deltaAcc
            if event.type == KEYDOWN and (event.key == K_LEFT):
                pipeVelX += deltaVel
                #playerAccX += deltaAcc
            if event.type == KEYDOWN and (event.key == K_RIGHT):
                pipeVelX -= deltaVel
                #playerAccX -= deltaAcc
            if event.type == pygame.USEREVENT and score > 0:
                if countdown > 0:
                    countdown -= 1
                else:
                    return {
                        'y': playery,
                        'groundCrash': crashTest[1],
                        'basex': basex,
                        'upperPipes': upperPipes,
                        'lowerPipes': lowerPipes,
                        'score': score,
                        'playerVelY': playerVelY,
                        'timeRanOut': 1,
                    }

        #update velocity
        pipeVelX += playerAccX
        playerVelY += playerAccY

        #limit velocity
        playerVelY = limitVel(playerVelY,1)
        pipeVelX = limitVel(pipeVelX,0)

        #publish pub_velocity
        pub_velocity.publish(Vector3(-SCALING*FPS*pipeVelX,-SCALING*FPS*playerVelY,0))

        # check for crash here
        crashTest = checkCrash({'x': playerx, 'y': playery, 'index': playerIndex},
                               upperPipes, lowerPipes)
        if crashTest[0]:
            return {
                'y': playery,
                'groundCrash': crashTest[1],
                'basex': basex,
                'upperPipes': upperPipes,
                'lowerPipes': lowerPipes,
                'score': score,
                'playerVelY': playerVelY,
                'timeRanOut': 0,
            }

        # check for score
        playerMidPos = playerx + IMAGES['player'][0].get_width() / 2
        pipeCounter = 0
        for pipe in upperPipes:
            pipeMidPos = pipe['x'] + IMAGES['pipe'][0].get_width() / 2
            if pipeMidPos <= playerMidPos < (pipeMidPos + IMAGES['pipe'][0].get_width() / 2):
                pipeCounter += 1
                if betweenPipes == 0:
                    score += 1
                    betweenPipes = 1
                    SOUNDS['point'].play()
        if pipeCounter == 0:
            betweenPipes = 0
        # playerIndex basex change
        if (loopIter + 1) % 3 == 0:
            playerIndex = next(playerIndexGen)
        loopIter = (loopIter + 1) % 30
        basex = -((-basex - pipeVelX) % baseShift)

        playerHeight = IMAGES['player'][playerIndex].get_height()
        playery += min(playerVelY, BASEY - playery - playerHeight)

        # move pipes to left
        for uPipe, lPipe in zip(upperPipes, lowerPipes):
            uPipe['x'] += pipeVelX
            lPipe['x'] += pipeVelX

        # add new pipe when first pipe each ? pixels
        if (SCREENWIDTH + 200) > upperPipes[-1]['x']:
            newPipe = getRandomPipe()
            newPipe[0]['x'] = PIPESPACING + upperPipes[-1]['x']
            newPipe[1]['x'] = PIPESPACING + upperPipes[-1]['x']
            upperPipes.append(newPipe[0])
            lowerPipes.append(newPipe[1])

        # remove first pipe if its out of the screen
        if upperPipes[0]['x'] < -IMAGES['pipe'][0].get_width():
            upperPipes.pop(0)
            lowerPipes.pop(0)

        # draw sprites
        SCREEN.blit(IMAGES['background'], (0,0))

        for uPipe, lPipe in zip(upperPipes, lowerPipes):
            SCREEN.blit(IMAGES['pipe'][0], (uPipe['x'], uPipe['y']))
            SCREEN.blit(IMAGES['pipe'][1], (lPipe['x'], lPipe['y']))

        SCREEN.blit(IMAGES['base'], (basex, BASEY))
        ###################################################################
        # get bitmap of obstacles
        bitmap = getBitmap(upperPipes,lowerPipes,(basex, BASEY))
        # do raytracing with Laser
        playerMiddle = (playerx + IMAGES['player'][0].get_width() / 2,playery + IMAGES['player'][0].get_height() / 2)
        laserPoints = laser.scan(playerMiddle,bitmap)

        # display
        if DEBUG == 1:
            # display obstacles and ray tracing
            bitmap = pygame.surfarray.make_surface(bitmap)
            SCREEN.blit(bitmap, (0, 0))

        for i in range(0,len(laserPoints)):
            if laserPoints[i][2] == 1:
                pygame.draw.circle(SCREEN,(0,255,0),laserPoints[i][0:2],3,0)
                pygame.draw.aaline(SCREEN,(0,255,0),playerMiddle,laserPoints[i][0:2],1)
            else :
                pygame.draw.aaline(SCREEN,(0,140,0),playerMiddle,laserPoints[i][0:2],1)
        ###################################################################

        # print score so player overlaps the score
        showScore(score)
        if score > 0:
            showCounter(countdown)
        playerSurface = pygame.transform.rotate(IMAGES['player'][playerIndex], 0)
        SCREEN.blit(playerSurface, (playerx, playery))

        pygame.display.update()
        FPSCLOCK.tick(FPS)


def showGameOverScreen(crashInfo):
    """crashes the player down ans shows gameover image"""
    score = crashInfo['score']
    playerx = SCREENWIDTH * 0.13
    playery = crashInfo['y']
    playerHeight = IMAGES['player'][0].get_height()
    playerVelY = crashInfo['playerVelY']
    playerAccY = 2

    basex = crashInfo['basex']

    upperPipes, lowerPipes = crashInfo['upperPipes'], crashInfo['lowerPipes']

    # play hit and die sounds
    if not crashInfo['timeRanOut']:
        SOUNDS['hit'].play()
    if not crashInfo['groundCrash']:
        SOUNDS['die'].play()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN and (event.key == K_SPACE or event.key == K_UP):
                if playery + playerHeight >= BASEY - 1:
                    return

        # player y shift
        if playery + playerHeight < BASEY - 1:
            playery += min(playerVelY, BASEY - playery - playerHeight)

        # player velocity change
        if playerVelY < 15:
            playerVelY += playerAccY

        # draw sprites
        SCREEN.blit(IMAGES['background'], (0,0))

        for uPipe, lPipe in zip(upperPipes, lowerPipes):
            SCREEN.blit(IMAGES['pipe'][0], (uPipe['x'], uPipe['y']))
            SCREEN.blit(IMAGES['pipe'][1], (lPipe['x'], lPipe['y']))

        SCREEN.blit(IMAGES['base'], (basex, BASEY))
        showScore(score)

        playerSurface = pygame.transform.rotate(IMAGES['player'][1], -45)
        SCREEN.blit(playerSurface, (playerx,playery))

        FPSCLOCK.tick(FPS)
        pygame.display.update()


def playerShm(playerShm):
    """oscillates the value of playerShm['val'] between 8 and -8"""
    if abs(playerShm['val']) == 8:
        playerShm['dir'] *= -1

    if playerShm['dir'] == 1:
         playerShm['val'] += 1
    else:
        playerShm['val'] -= 1


def getRandomPipe():
    """returns a randomly generated pipe"""
    # y of gap between upper and lower pipe
    gapY = random.randrange(0, int(BASEY * 0.6 - PIPEGAPSIZE))
    gapY += int(BASEY * 0.2)
    pipeHeight = IMAGES['pipe'][0].get_height()
    pipeX = SCREENWIDTH + 10

    return [
        {'x': pipeX, 'y': gapY - pipeHeight},  # upper pipe
        {'x': pipeX, 'y': gapY + PIPEGAPSIZE}, # lower pipe
    ]


def showScore(score):
    """displays score in center of screen"""
    scoreDigits = [int(x) for x in list(str(score))]
    totalWidth = 0 # total width of all numbers to be printed

    for digit in scoreDigits:
        totalWidth += IMAGES['numbers'][digit].get_width()

    Xoffset = (SCREENWIDTH - totalWidth) / 2

    for digit in scoreDigits:
        SCREEN.blit(IMAGES['numbers'][digit], (Xoffset, SCREENHEIGHT * 0.1))
        Xoffset += IMAGES['numbers'][digit].get_width()

def showCounter(counter):
    """displays score in center of screen"""
    scoreDigits = [int(x) for x in list(str(counter))]
    totalWidth = 0 # total width of all numbers to be printed

    for digit in scoreDigits:
        totalWidth += IMAGES['numbers'][digit].get_width()

    Xoffset = (SCREENWIDTH - totalWidth) / 2

    for digit in scoreDigits:
        SCREEN.blit(IMAGES['numbers'][digit], (Xoffset, SCREENHEIGHT * 0.85))
        Xoffset += IMAGES['numbers'][digit].get_width()

def checkCrash(player, upperPipes, lowerPipes):
    """returns True if player collders with top, base or pipes."""
    pi = player['index']
    player['w'] = IMAGES['player'][0].get_width()
    player['h'] = IMAGES['player'][0].get_height()

    # if player crashes into ground
    if player['y'] + player['h'] >= BASEY - 1:
        return [True, True]
    elif player['y'] <= 0:
        return [True, True]
    else:

        playerRect = pygame.Rect(player['x'], player['y'],
                      player['w'], player['h'])
        pipeW = IMAGES['pipe'][0].get_width()
        pipeH = IMAGES['pipe'][0].get_height()

        for uPipe, lPipe in zip(upperPipes, lowerPipes):
            # upper and lower pipe rects
            uPipeRect = pygame.Rect(uPipe['x'], uPipe['y'], pipeW, pipeH)
            lPipeRect = pygame.Rect(lPipe['x'], lPipe['y'], pipeW, pipeH)

            # player and upper/lower pipe hitmasks
            pHitMask = HITMASKS['player'][pi]
            uHitmask = HITMASKS['pipe'][0]
            lHitmask = HITMASKS['pipe'][1]

            # if bird collided with upipe or lpipe
            uCollide = pixelCollision(playerRect, uPipeRect, pHitMask, uHitmask)
            lCollide = pixelCollision(playerRect, lPipeRect, pHitMask, lHitmask)

            if uCollide or lCollide:
                return [True, False]

    return [False, False]

def pixelCollision(rect1, rect2, hitmask1, hitmask2):
    """Checks if two objects collide and not just their rects"""
    rect = rect1.clip(rect2)

    if rect.width == 0 or rect.height == 0:
        return False

    x1, y1 = rect.x - rect1.x, rect.y - rect1.y
    x2, y2 = rect.x - rect2.x, rect.y - rect2.y

    for x in xrange(rect.width):
        for y in xrange(rect.height):
            if hitmask1[x1+x][y1+y] and hitmask2[x2+x][y2+y]:
                return True
    return False

def getHitmask(image):
    """returns a hitmask using an image's alpha."""
    mask = []
    for x in xrange(image.get_width()):
        mask.append([])
        for y in xrange(image.get_height()):
            mask[x].append(bool(image.get_at((x,y))[3]))
    return mask

def getBitmap(upperPipes,lowerPipes,base):
    obstacleSurface = pygame.Surface((SCREENWIDTH, SCREENHEIGHT),flags=pygame.SRCALPHA)
    obstacleSurface.fill((0,0,0,0))
    #Add obstacles
    for uPipe, lPipe in zip(upperPipes, lowerPipes):
        obstacleSurface.blit(IMAGES['pipe'][0], (uPipe['x'], uPipe['y']))
        obstacleSurface.blit(IMAGES['pipe'][1], (lPipe['x'], lPipe['y']))
        obstacleSurface.blit(IMAGES['base'], base)

    #Copy alpha values
    #bitmap = pygame.mask.from_surface(obstacleSurface,255)
    bitmap = surfarray.array_alpha(obstacleSurface)
    return bitmap

def controlCallback(data):
    global playerAccX
    global playerAccY

    playerAccX = limitAcceleration(-data.x,ACCXLIMIT)/(FPS*FPS*SCALING)
    playerAccY = limitAcceleration(-data.y,ACCYLIMIT)/(FPS*FPS*SCALING)
    #print(playerAccY)

def limitAcceleration(accUser,limit):
    if accUser > limit:
        accUser = limit
    elif accUser < -limit:
        accUser = -limit
    return accUser

def limitVel(velUser,direction):
    if velUser > VELLIMIT and direction == 1:
        velUser = VELLIMIT
    elif velUser > 0 and direction == 0:
        velUser = 0
    elif velUser < -VELLIMIT:
        velUser = -VELLIMIT
    return velUser

def shutdownHoook():
    print("Shutting flappy down!!!")
    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
