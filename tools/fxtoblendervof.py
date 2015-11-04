#!/usr/bin/python3

import math
import sys

fx = 367.06039
res_x = 512

if (len(sys.argv) == 3):
    fx = float(sys.argv[1])
    res_x = int(sys.argv[2])

fov_rad = 2*math.atan(res_x/(2*fx))
fov = math.degrees(fov_rad)

print("fx: ", fx)
print("X-Resolution: ", res_x)
print("FOV-radians: ", fov_rad)
print("FOV-degrees: ", fov)
