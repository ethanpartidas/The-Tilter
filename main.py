import math
import time

import serial
from PIL import ImageGrab

# Video feed position
small_x = 70
small_y = 280
big_x = small_x + 490
big_y = small_y + 490

# Coefficients
K_p = 0.5  # Position
K_d = 0.6  # Derivative
K_i = 0.5  # Integral
force_smoothing = 0.5  # Helps reduce jitter


# noinspection PyShadowingNames
def drawBoundBox():  # For *testing*
    testImg = ImageGrab.grab()
    pixels = testImg.load()

    for y in range(small_y, big_y):
        for x in range(small_x, big_x):
            # noinspection PyUnresolvedReferences
            pixels[x, y] = (0, 255, 0)

    testImg.show()


def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)


# Serial connection
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM3'
ser.open()


def set_motor(index, angle):
    ser.write(
        bytes(str(chr(ord('a') + index)), 'ascii') + bytes(str(angle // 100), 'ascii') + bytes(str(angle // 10 % 10),
                                                                                               'ascii') + bytes(
            str(angle % 10), 'ascii'))


# noinspection PyShadowingNames
def set_force(x, y):
    x = clamp(x, -1, 1)
    y = clamp(y, -1, 1)

    set_motor(0, int(180 / math.pi * math.asin((-y + 1) / 2)))
    set_motor(1, int(180 / math.pi * math.asin((clamp((math.sqrt(3) / 2 * -x + 1 / 2 * y), -1, 1) + 1) / 2)))
    set_motor(2, int(180 / math.pi * math.asin((clamp((math.sqrt(3) / 2 * x + 1 / 2 * y), -1, 1) + 1) / 2)))


if __name__ == "__main__":
    time.sleep(3)

    last_x = last_y = 0
    lastTime = time.time()

    int_x = int_y = 0
    last_force_x = last_force_y = 0
    while True:
        im = ImageGrab.grab()
        px = im.load()

        x_sum = y_sum = total = 0
        for y in range(small_y, big_y):
            for x in range(small_x, big_x):
                # noinspection PyUnresolvedReferences
                if px[x, y][2] >= 230:
                    x_sum += x
                    y_sum += y
                    total += 1

        if total <= 10:
            continue

        x_range, y_range = big_x - small_x, big_y - small_y
        avg_x, avg_y = x_sum / total, y_sum / total
        normal_x, normal_y = (avg_x - small_x) / x_range, (avg_y - small_y) / y_range
        coord_x, coord_y, current_time = 2 * normal_x - 1, -2 * normal_y + 1, time.time()

        target_x, target_y = 0, 0
        error_x, error_y = target_x - coord_x, target_y - coord_y
        dt = current_time - lastTime
        d_x, d_y = (error_x - last_x) / dt, (error_y - last_y) / dt
        last_x, last_y, lastTime = error_x, error_y, current_time
        int_x += error_x * dt
        int_y += error_y * dt
        int_x = clamp(int_x, -1, 1)
        int_y = clamp(int_y, -1, 1)

        force_x = (K_p * error_x + K_d * d_x + K_i * int_x) * (1 - force_smoothing) + last_force_x * force_smoothing
        last_force_x = force_x
        force_y = (K_p * error_y + K_d * d_y + K_i * int_y) * (1 - force_smoothing) + last_force_y * force_smoothing
        last_force_y = force_y

        print(f"Proportional: {error_x:.3f}, {error_y:.3f}", end='\t')
        print(f"Derivative: {d_x:.3f}, {d_y:.3f}", end='\t')
        print(f"Integral: {int_x:.3f}, {int_y:.3f}", end=' \t')
        print(f"Force: {force_x:.3f}, {force_y:.3f}")
        set_force(force_x, force_y)
