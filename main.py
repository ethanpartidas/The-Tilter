import numpy
from PIL import ImageGrab, ImageShow, Image
import math
import serial
import time
import cv2

time.sleep(3)
# ImageGrab.grab().save("Screenie.png")
smallX = 70
smallY = 280
bigX = smallX + 490
bigY = smallY + 490
K_p = 0.5
K_d = 0.6
K_i = 0.5
forceSmoothing = 0.5
# im.show()
# im.save("Screenie.png")

def drawBoundBox():
    testImg = ImageGrab.grab()
    pixels = testImg.load()

    for y in range(smallY, bigY):
        for x in range(smallX, bigX):
            # if pixels[x, y][2] >= 230:
            pixels[x, y] = (0, 255, 0)

    testImg.show()


# drawBoundBox()

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

def main():
    lastX, lastY, lastTime = 0, 0, time.time()
    intX, intY = 0, 0
    lastFx, lastFy = 0, 0
    while True:
        im = ImageGrab.grab()
        # width, height = im.size
        px = im.load()
        # px[x, y] = (0, 255, 0)

        xsum, ysum, total = 0, 0, 0

        for y in range(smallY, bigY):
            for x in range(smallX, bigX):
                if px[x, y][2] >= 230:
                    xsum += x
                    ysum += y
                    total += 1

        if total <= 10:
            continue

        xrange, yrange = bigX - smallX, bigY - smallY
        avgX, avgY = xsum / total, ysum / total
        normalX, normalY = (avgX - smallX) / xrange, (avgY - smallY) / yrange
        coordX, coordY, current_time = 2*normalX - 1, -2*normalY + 1, time.time()

        # targetX, targetY = math.cos(4*time.time())/5, math.sin(4*time.time())/5
        # targetX, targetY = math.cos(time.time())/10, 0
        targetX, targetY = 0, 0
        errorX, errorY = targetX - coordX, targetY - coordY
        dt = current_time - lastTime
        dX, dY = (errorX - lastX)/dt, (errorY - lastY)/dt
        lastX, lastY, lastTime = errorX, errorY, current_time
        intX += errorX * dt
        intY += errorY * dt
        intX = clamp(intX, -1, 1)
        intY = clamp(intY, -1, 1)
        # speed = 2
        # if dX * dX + dY * dY > speed * speed:
        #     intX = 0
        #     intY = 0
        #     print('reset integral')
        forceX = (K_p * errorX + K_d * dX + K_i * intX) * (1-forceSmoothing) + lastFx * forceSmoothing
        lastFx = forceX
        forceY = (K_p * errorY + K_d * dY + K_i * intY) * (1-forceSmoothing) + lastFy * forceSmoothing
        lastFy = forceY

        print(f"Proportional: {errorX:.3f}, {errorY:.3f}", end='\t')
        print(f"Derivative: {dX:.3f}, {dY:.3f}", end='\t')
        print(f"Integral: {intX:.3f}, {intY:.3f}", end=' \t')
        print(f"Force: {forceX:.3f}, {forceY:.3f}")
        setforce(forceX, forceY)

        # numpy_img = numpy.array(im.getdata(), dtype='uint8').reshape(im.size[1], im.size[0], 3)
        # cv_im = cv2.resize(numpy_img, (300, 300))
        # cv2.imshow("Display", cv_im)

        # time.sleep(0.05)

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM3'
ser.open()

# max_angle = 90

def setmotor(index, angle):
    # angle = clamp(angle, 0, max_angle)
    ser.write(bytes(str(chr(ord('a')+index)), 'ascii') + bytes(str(angle//100), 'ascii') + bytes(str(angle//10%10), 'ascii') + bytes(str(angle%10), 'ascii'))

def setforce(x, y):
    x = clamp(x, -1, 1)
    y = clamp(y, -1, 1)
    setmotor(0, int(180 / math.pi * math.asin((-y+1)/2)))
    setmotor(1, int(180 / math.pi * math.asin((clamp((math.sqrt(3)/2 * -x + 1/2 * y), -1, 1)+1)/2)))
    setmotor(2, int(180 / math.pi * math.asin((clamp((math.sqrt(3)/2 * x + 1/2 * y), -1, 1)+1)/2)))
    # setmotor(0, int(max_angle * (-y+1)/2))
    # setmotor(1, int(max_angle * (clamp((math.sqrt(3)/2 * -x + 1/2 * y), -1, 1)+1)/2))
    # setmotor(2, int(max_angle * (clamp((math.sqrt(3)/2 * x + 1/2 * y), -1, 1)+1)/2))

main()
