import matplotlib.pyplot as plt
import csv
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams['legend.fontsize'] = 10

time = []
X1 = []
X2 = []
X3 = []
Y1 = []
Y2 = []
Y3 = []
Z1 = []
Z2 = []
Z3 = []
Yawx1 = []
Yawy1 = []
arx1 = []
ary1 = []
Yawx2 = []
Yawy2 = []
arx2 = []
ary2 = []
Yawx3 = []
Yawy3 = []
arx3 = []
ary3 = []

it = 0
arrow = 100

left = (2.0-0.54641)/4.0
bottom = (2.0-0.14641)/4.0
height = 0.8/4.0
width = 0.8/4.0
right = left + width
top = bottom + height

with open('_slash_position_controller_leader_data.csv','r') as csvfile:
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

with open('_slash_position_controller_follower1_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	it=it+1
        time.append(int(row[0]))
	X2.append(float(row[7]))
	Y2.append(float(row[8]))
	Z2.append(float(row[9]))
	if it==arrow:
		arx2.append(float(row[7]))
		ary2.append(float(row[8]))
		Yawx2.append(  np.cos(float(row[10])*np.pi/180.0)   )
		Yawy2.append(  np.sin(float(row[10])*np.pi/180.0)   )
		it=0

with open('_slash_position_controller_follower2_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	it=it+1
        time.append(int(row[0]))
	X3.append(float(row[7]))
	Y3.append(float(row[8]))
	Z3.append(float(row[9]))
	if it==arrow:
		arx3.append(float(row[7]))
		ary3.append(float(row[8]))
		Yawx3.append(  np.cos(float(row[10])*np.pi/180.0)   )
		Yawy3.append(  np.sin(float(row[10])*np.pi/180.0)   )
		it=0

marker_size=10
marker_size2=6		
marker_start = [0] 
marker_stop1 = [len(X1)-1]
marker_stop2 = [len(X2)-1]
marker_stop3 = [len(X3)-1]
fig = plt.figure()

ax = fig.add_subplot(111)
ax.plot(X1,Y1,'r',label='Trajektoria L', linewidth=2)
ax.plot(X2,Y2,'b',label='Trajektoria F1', linewidth=2)
ax.plot(X3,Y3,'g',label='Trajektoria F2', linewidth=2)
ax.plot(X1,Y1,'r^',markevery=marker_start, markersize=marker_size, label='Pkt startowy L')
ax.plot(X1,Y1,'ro',markevery=marker_stop1, markersize=marker_size, label='Pkt koncowy L')
ax.plot(X2,Y2,'b^',markevery=marker_start, markersize=marker_size, label='Pkt startowy F1')
ax.plot(X2,Y2,'bo',markevery=marker_stop2, markersize=marker_size, label='Pkt koncowy F1')
ax.plot(X3,Y3,'g^',markevery=marker_start, markersize=marker_size, label='Pkt startowy F2')
ax.plot(X3,Y3,'go',markevery=marker_stop3, markersize=marker_size, label='Pkt koncowy F2')

ax.quiver(arx1,ary1,Yawx1,Yawy1, scale=30, width=0.0007, headwidth=15, headlength=15, color='#ff7070') #red
ax.quiver(arx2,ary2,Yawx2,Yawy2, scale=30, width=0.0007, headwidth=15, headlength=15, color='#8c90ff') #blue
ax.quiver(arx3,ary3,Yawx3,Yawy3, scale=30, width=0.0007, headwidth=15, headlength=15, color='#6df980') #green

ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.grid(True)
ax.legend(numpoints=1)

p = patches.Rectangle( (left, bottom), width, height, angle=330.0,
    fill=True, transform=ax.transAxes, facecolor='#e8deb2', clip_on=True, edgecolor='black')
ax.add_patch(p)
ax.text(0.5, 0.5, 'Obiekt',
        horizontalalignment='center',
        verticalalignment='center',
        fontsize=20, color='black',
        transform=ax.transAxes)

plt.show()
