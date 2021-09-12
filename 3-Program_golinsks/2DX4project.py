# Sam Golinsky
# 400176564
# Python 3.7.7
# libraries: serial, math, numpy and open3d

import serial
import math
import numpy as np
import open3d as o3d

# converts distance to x, y, z
def convert_dist(): 
    global j
    global x_dist
    a = 0 # angle of motor in quadrant
    for e in entry_scan:
        distance, status = e.split(', ')
        distance = int(distance)
        status = int(status)
        if status == 0 or status == 1 or status == 2: # when no error append x, y, z to data array
            if j == 0: # direct +z
                data.append(f"{x_dist}, 0, {distance}")
            elif 0 < j < 8: # first quadrant
                data.append(f"{x_dist}, {distance * math.sin(math.radians(a))}, {distance * math.cos(math.radians(a))}")
            elif j == 8: # direct +y
                data.append(f"{x_dist}, {distance}, 0")
            elif 8 < j < 16: # second quadrant
                data.append(
                    f"{x_dist}, {distance * math.cos(math.radians(a))}, {-distance * math.sin(math.radians(a))}")
            elif j == 16: # direct -z
                data.append(f"{x_dist}, 0, {-distance}")
            elif 16 < j < 24: # third quadrant
                data.append(
                    f"{x_dist}, {-distance * math.sin(math.radians(a))}, {-distance * math.cos(math.radians(a))}")
            elif j == 24: # direct -y
                data.append(f"{x_dist}, {-distance}, 0")
            elif 24 < j < 32: # fourth quadrant
                data.append(
                    f"{x_dist}, {-distance * math.cos(math.radians(a))}, {distance * math.sin(math.radians(a))}")
        if status == 4 or status == 7: # when error in distance append previous value
            data.append(data[-1])
        a += 11.25 # increment angle
        j += 1
        if j % 8 == 0 and j > 0: # when done first quadrant, reset angle
            a = 0
    return


i = 0
j = 0
x_dist = 0
entry = ''
entry_scan = []
data = []

s = serial.Serial("COM8", 115200)

print("Opening: " + s.name)

while 1:
    x = s.read()  # read one byte
    c = x.decode()  # convert byte type to str
    if c != '\n': # corresponds to what was sent from keil. Basically only take distance and status
        entry += c
    else:
        print(entry)
        entry_scan.append(entry) # appends "{distance}, {status}"
        i += 1
        entry = '' # resets entry for next distance and status

    if i % 32 == 0 and i > 0 and entry_scan != []: # after one scan done we convert to x, y, z
        convert_dist() # convert to x, y, z
        x_dist += 200 # increment x coord
        j = 0
        entry_scan = [] # empty for next scan
        print(data)

    if i == 320: # when all datapoints for 10 btn presses
        print("Closing: " + s.name)
        s.close()
        break

f = open("data_file.xyz", "w")

for p in data:
    x, y, z = p.split(', ')
    f.write(f"{x} {y} {z}\n") # write data into file in correct format for open3d
f.close()

print("Testing IO fort point cloud...")
pcd = o3d.io.read_point_cloud("data_file.xyz", format='xyz')

print(pcd)

print(np.asarray(pcd.points))

pt1 = 0
pt2 = 1
pt3 = 2
pt4 = 3
pt5 = 4
pt6 = 5
pt7 = 6
pt8 = 7
pt9 = 8
pt10 = 9
pt11 = 10
pt12 = 11
pt13 = 12
pt14 = 13
pt15 = 14
pt16 = 15
pt17 = 16
pt18 = 17
pt19 = 18
pt20 = 19
pt21 = 20
pt22 = 21
pt23 = 22
pt24 = 23
pt25 = 24
pt26 = 25
pt27 = 26
pt28 = 27
pt29 = 28
pt30 = 29
pt31 = 30
pt32 = 31
po = 0

lines = []

for x in range(10): # connects first points to second, second to third ... last to first
    lines.append([pt1 + po, pt2 + po])
    lines.append([pt2 + po, pt3 + po])
    lines.append([pt3 + po, pt4 + po])
    lines.append([pt4 + po, pt5 + po])
    lines.append([pt5 + po, pt6 + po])
    lines.append([pt6 + po, pt7 + po])
    lines.append([pt7 + po, pt8 + po])
    lines.append([pt8 + po, pt9 + po])
    lines.append([pt9 + po, pt10 + po])
    lines.append([pt10 + po, pt11 + po])
    lines.append([pt11 + po, pt12 + po])
    lines.append([pt12 + po, pt13 + po])
    lines.append([pt13 + po, pt14 + po])
    lines.append([pt14 + po, pt15 + po])
    lines.append([pt15 + po, pt16 + po])
    lines.append([pt16 + po, pt17 + po])
    lines.append([pt17 + po, pt18 + po])
    lines.append([pt18 + po, pt19 + po])
    lines.append([pt19 + po, pt20 + po])
    lines.append([pt20 + po, pt21 + po])
    lines.append([pt21 + po, pt22 + po])
    lines.append([pt22 + po, pt23 + po])
    lines.append([pt23 + po, pt24 + po])
    lines.append([pt24 + po, pt25 + po])
    lines.append([pt25 + po, pt26 + po])
    lines.append([pt26 + po, pt27 + po])
    lines.append([pt27 + po, pt28 + po])
    lines.append([pt28 + po, pt29 + po])
    lines.append([pt29 + po, pt30 + po])
    lines.append([pt30 + po, pt31 + po])
    lines.append([pt31 + po, pt32 + po])
    lines.append([pt32 + po, pt1 + po])
    po += 32

pt1 = 0
pt2 = 1
pt3 = 2
pt4 = 3
pt5 = 4
pt6 = 5
pt7 = 6
pt8 = 7
pt9 = 8
pt10 = 9
pt11 = 10
pt12 = 11
pt13 = 12
pt14 = 13
pt15 = 14
pt16 = 15
pt17 = 16
pt18 = 17
pt19 = 18
pt20 = 19
pt21 = 20
pt22 = 21
pt23 = 22
pt24 = 23
pt25 = 24
pt26 = 25
pt27 = 26
pt28 = 27
pt29 = 28
pt30 = 29
pt31 = 30
pt32 = 31
po = 0
do = 32

for x in range(9): # connects all the scans together 
    lines.append([pt1 + po, pt1 + do + po])
    lines.append([pt2 + po, pt2 + do + po])
    lines.append([pt3 + po, pt3 + do + po])
    lines.append([pt4 + po, pt4 + do + po])
    lines.append([pt5 + po, pt5 + do + po])
    lines.append([pt6 + po, pt6 + do + po])
    lines.append([pt7 + po, pt7 + do + po])
    lines.append([pt8 + po, pt8 + do + po])
    lines.append([pt9 + po, pt9 + do + po])
    lines.append([pt10 + po, pt10 + do + po])
    lines.append([pt11 + po, pt11 + do + po])
    lines.append([pt12 + po, pt12 + do + po])
    lines.append([pt13 + po, pt13 + do + po])
    lines.append([pt14 + po, pt14 + do + po])
    lines.append([pt15 + po, pt15 + do + po])
    lines.append([pt16 + po, pt16 + do + po])
    lines.append([pt17 + po, pt17 + do + po])
    lines.append([pt18 + po, pt18 + do + po])
    lines.append([pt19 + po, pt19 + do + po])
    lines.append([pt20 + po, pt20 + do + po])
    lines.append([pt21 + po, pt21 + do + po])
    lines.append([pt22 + po, pt22 + do + po])
    lines.append([pt23 + po, pt23 + do + po])
    lines.append([pt24 + po, pt24 + do + po])
    lines.append([pt25 + po, pt25 + do + po])
    lines.append([pt26 + po, pt26 + do + po])
    lines.append([pt27 + po, pt27 + do + po])
    lines.append([pt28 + po, pt28 + do + po])
    lines.append([pt29 + po, pt29 + do + po])
    lines.append([pt30 + po, pt30 + do + po])
    lines.append([pt31 + po, pt31 + do + po])
    lines.append([pt32 + po, pt32 + do + po])
    po += 32

line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                                lines=o3d.utility.Vector2iVector(lines)) # set the points and lines together to visualize next
o3d.visualization.draw_geometries([line_set]) # visualize 

