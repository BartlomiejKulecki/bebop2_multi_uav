import matplotlib.pyplot as plt
import csv
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams['legend.fontsize'] = 10

time = []
X1 = []
Y1 = []
Z1 = []
Yawx1 = []
Yawy1 = []
arx1 = []
ary1 = []

it = 0
arrow = 100

left = (1.5-0.4)/3.0
bottom = (1.5-0.4)/3.0
height = 0.8/3.0
width = 0.8/3.0
right = left + width
top = bottom + height

with open('_slash_position_controller_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	it=it+1
        time.append(int(row[0]))
	X1.append(float(row[7]))
	Y1.append(float(row[8]))
	Z1.append(float(row[9]))
	if it==arrow:
		arx1.append(float(row[7]))
		ary1.append(float(row[8]))
		Yawx1.append(  np.cos(float(row[10])*np.pi/180.0)   )
		Yawy1.append(  np.sin(float(row[10])*np.pi/180.0)   )
		it=0


marker_size=10	
marker_start = [0] 
marker_stop = [len(X1)-1]

fig = plt.figure()

ax = fig.add_subplot(111)
ax.plot(X1,Y1,'r', linewidth=2, label='Trajektoria aktualna')
ax.plot(X1,Y1,'r^',markevery=marker_start, markersize=marker_size, label='Punkt startowy')
ax.plot(X1,Y1,'ro',markevery=marker_stop, markersize=marker_size, label='Punkt koncowy')

ax.quiver(arx1,ary1,Yawx1,Yawy1, scale=30, width=0.0007, headwidth=15, headlength=15, color='#ff7070') #red

ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.grid(True)
ax.legend(numpoints=1)

p = patches.Rectangle( (left, bottom), width, height,
    fill=True, transform=ax.transAxes, facecolor='#e8deb2', clip_on=True, edgecolor='black')
ax.add_patch(p)
ax.text(0.5*(left+right), 0.5*(bottom+top), 'Obiekt',
        horizontalalignment='center',
        verticalalignment='center',
        fontsize=20, color='black',
        transform=ax.transAxes)

plt.show()
