import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams['legend.fontsize'] = 10

time = []
Xt = []
Xact = []
Yt = []
Yact = []
Zt = []
Zact = []


with open('_slash_position_controller_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
        time.append(int(row[0]))
        Xt.append(float(row[2]))
	Yt.append(float(row[3]))
	Zt.append(float(row[4]))
	Xact.append(float(row[7]))
	Yact.append(float(row[8]))
	Zact.append(float(row[9]))

marker_size=10	
marker_start = [0] 
marker_stop = [len(Xact)-1]

fig = plt.figure()
ax = fig.add_subplot(221,projection='3d')
ax.plot(Xt,Yt,Zt,'b', label='Trajektoria referencyjna', linewidth=2)
ax.plot(Xact,Yact,Zact,'r', label='Trajektoria aktualna', linewidth=2)
ax.plot(Xact,Yact,Zact,'r^',markevery=marker_start, markersize=marker_size, label='Punkt startowy')
ax.plot(Xact,Yact,Zact,'ro',markevery=marker_stop, markersize=marker_size, label='Punkt koncowy')
ax.legend(numpoints=1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.grid(True)

ax = fig.add_subplot(222)
ax.plot(Yt,Zt,'b')
ax.plot(Yact,Zact,'r')
ax.plot(Yact,Zact,'r^',markevery=marker_start, markersize=marker_size)
ax.plot(Yact,Zact,'ro',markevery=marker_stop, markersize=marker_size)
ax.set_ylabel('Z')
ax.set_xlabel('Y')
ax.grid(True)

ax = fig.add_subplot(223)
ax.plot(Xt,Zt,'b')
ax.plot(Xact,Zact,'r')
ax.plot(Xact,Zact,'r^',markevery=marker_start, markersize=marker_size)
ax.plot(Xact,Zact,'ro',markevery=marker_stop, markersize=marker_size)
ax.set_ylabel('Z')
ax.set_xlabel('X')
ax.grid(True)

ax = fig.add_subplot(224)
ax.plot(Xt,Yt,'b')
ax.plot(Xact,Yact,'r')
ax.plot(Xact,Yact,'r^',markevery=marker_start, markersize=marker_size)
ax.plot(Xact,Yact,'ro',markevery=marker_stop, markersize=marker_size)
ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.grid(True)
ax.legend()


plt.show()
