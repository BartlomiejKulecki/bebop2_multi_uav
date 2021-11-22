import matplotlib.pyplot as plt
import csv
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


with open('_slash_position_controller_leader_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
        time.append(int(row[0]))
	X1.append(float(row[7]))
	Y1.append(float(row[8]))
	Z1.append(float(row[9]))
	
with open('_slash_position_controller_follower1_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
        time.append(int(row[0]))
	X2.append(float(row[7]))
	Y2.append(float(row[8]))
	Z2.append(float(row[9]))

with open('_slash_position_controller_follower2_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
        time.append(int(row[0]))
	X3.append(float(row[7]))
	Y3.append(float(row[8]))
	Z3.append(float(row[9]))

marker_size=10	
marker_start = [0] 
marker_stop1 = [len(X1)-1]
marker_stop2 = [len(X2)-1]
marker_stop3 = [len(X3)-1]

fig = plt.figure()
ax = fig.add_subplot(221,projection='3d')
ax.plot(X1,Y1,Z1,'r', label='Trajektoria L')
ax.plot(X2,Y2,Z2,'b', label='Trajektoria F1')
ax.plot(X3,Y3,Z3,'g', label='Trajektoria F2')
ax.plot(X1,Y1,Z1,'r^',markevery=marker_start, markersize=marker_size)
ax.plot(X1,Y1,Z1,'ro',markevery=marker_stop1, markersize=marker_size)
ax.plot(X2,Y2,Z2,'b^',markevery=marker_start, markersize=marker_size)
ax.plot(X2,Y2,Z2,'bo',markevery=marker_stop2, markersize=marker_size)
ax.plot(X3,Y3,Z3,'g^',markevery=marker_start, markersize=marker_size)
ax.plot(X3,Y3,Z3,'go',markevery=marker_stop3, markersize=marker_size)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.grid(True)

ax = fig.add_subplot(222)
ax.plot(Y1,Z1,'r', label='Trajektoria L')
ax.plot(Y2,Z2,'b', label='Trajektoria F1')
ax.plot(Y3,Z3,'g', label='Trajektoria F2')
ax.plot(Y1,Z1,'r^',markevery=marker_start, markersize=marker_size, label='Pkt startowy L')
ax.plot(Y1,Z1,'ro',markevery=marker_stop1, markersize=marker_size, label='Pkt koncowy L')
ax.plot(Y2,Z2,'b^',markevery=marker_start, markersize=marker_size, label='Pkt startowy F1')
ax.plot(Y2,Z2,'bo',markevery=marker_stop2, markersize=marker_size, label='Pkt koncowy F1')
ax.plot(Y3,Z3,'g^',markevery=marker_start, markersize=marker_size, label='Pkt startowy F2')
ax.plot(Y3,Z3,'go',markevery=marker_stop3, markersize=marker_size, label='Pkt koncowy F2')
ax.set_ylabel('Z')
ax.set_xlabel('Y')
ax.legend(numpoints=1,loc='lower left')
ax.grid(True)

ax = fig.add_subplot(223)
ax.plot(X1,Z1,'r')
ax.plot(X2,Z2,'b')
ax.plot(X3,Z3,'g')
ax.plot(X1,Z1,'r^',markevery=marker_start, markersize=marker_size)
ax.plot(X1,Z1,'ro',markevery=marker_stop1, markersize=marker_size)
ax.plot(X2,Z2,'b^',markevery=marker_start, markersize=marker_size)
ax.plot(X2,Z2,'bo',markevery=marker_stop2, markersize=marker_size)
ax.plot(X3,Z3,'g^',markevery=marker_start, markersize=marker_size)
ax.plot(X3,Z3,'go',markevery=marker_stop3, markersize=marker_size)
ax.set_ylabel('Z')
ax.set_xlabel('X')
ax.grid(True)

ax = fig.add_subplot(224)
ax.plot(X1,Y1,'r')
ax.plot(X2,Y2,'b')
ax.plot(X3,Y3,'g')
ax.plot(X1,Y1,'r^',markevery=marker_start, markersize=marker_size)
ax.plot(X1,Y1,'ro',markevery=marker_stop1, markersize=marker_size)
ax.plot(X2,Y2,'b^',markevery=marker_start, markersize=marker_size)
ax.plot(X2,Y2,'bo',markevery=marker_stop2, markersize=marker_size)
ax.plot(X3,Y3,'g^',markevery=marker_start, markersize=marker_size)
ax.plot(X3,Y3,'go',markevery=marker_stop3, markersize=marker_size)
ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.grid(True)

plt.show()
