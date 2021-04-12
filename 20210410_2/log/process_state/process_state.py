from pyproj import Proj
import numpy as np
import pandas as pd
from pyproj import Transformer
import matplotlib.pyplot as plt

def WGS2UTM(lon,lat):
    #WGS84转UTM https://epsg.io/32650
    transformer = Transformer.from_crs("epsg:4326","epsg:32650")
    x,y = transformer.transform(lat,lon)
    return x,y

data = pd.read_csv('../State.csv')
lon = data.Longitude.values
lat = data.Latitude.values
height = data.Height.values
time = data.UnixTime.values
microt = data.Microseconds.values
roll = data.Roll.values
pitch = data.Pitch.values
yaw = data.Heading.values

x1,y1 = WGS2UTM(lon,lat)
x1 = x1-x1[0]
y1 = y1-y1[0]
st = 0
ed = 0
for i in range(0, len(time)):
    if time[i] == 1618044938:
        if microt[i]/1e6 > 260012/1e9:
            st = i
            break
for i in range(st, len(time)):
    if time[i] == 1618045174:
        if microt[i]/1e6 > 415152/1e9:
            ed = i
            break
cntt = 0
x2=[]
y2=[]
z2=[]
roll2=[]
pitch2=[]
yaw2=[]
for i in range(st, ed):
    if cntt%20 == 0:
        x2.append(x1[i])
        y2.append(y1[i])
        z2.append(height[i])
        roll2.append(roll[i])
        pitch2.append(pitch[i])
        yaw2.append(yaw[i])
        cntt = 0
    cntt = cntt+1
z2 = z2 - z2[0]
roll2 = roll2-roll2[0]
pitch2 = pitch2-pitch2[0]
yaw2 = yaw2-yaw2[0]
with open('x.txt', 'w') as fp:
    fp.write('\n'.join('%f' % x for x in x2))
with open('y.txt', 'w') as fp:
    fp.write('\n'.join('%f' % y for y in y2))
with open('z.txt', 'w') as fp:
    fp.write('\n'.join('%f' % z for z in z2))
with open('roll.txt', 'w') as fp:
    fp.write('\n'.join('%f' % r for r in roll2))
with open('pitch.txt', 'w') as fp:
    fp.write('\n'.join('%f' % p for p in pitch2))
with open('yaw.txt', 'w') as fp:
    fp.write('\n'.join('%f' % y for y in yaw2))

plt.figure(figsize=(8,8),dpi=80)
plot1 = plt.figure(1)
plt.plot(range(0,len(z2)),z2)
plt.xlabel("time")
plt.ylabel("height (m)")
plt.title("Height")

plt.figure(figsize=(8,8),dpi=80)
plot1 = plt.figure(2)
plt.plot(x1,y1)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Trajectory")
plt.axis('equal')

plt.show()