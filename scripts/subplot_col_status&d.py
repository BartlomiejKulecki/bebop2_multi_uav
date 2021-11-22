import matplotlib.pyplot as plt
import csv
import math as m
import matplotlib.ticker as tick
plt.rcParams['legend.fontsize'] = 12

dtime = []
d1 = []
d2 = []
d3 = []
status = []
h1 = []
h2 = []
h3 = []
r1 = []
r2 = []
r3 = []
tmp=0
Ra = []
Rz = []

with open('_slash_collision_avoidance_slash_d1.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	if tmp==0:
		zero = int(row[0])
		tmp = 1
        dtime.append( (int(row[0])-zero) )
	d1.append(float(row[1]))
with open('_slash_collision_avoidance_slash_d2.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	d2.append(float(row[1]))
with open('_slash_collision_avoidance_slash_d3.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	d3.append(float(row[1]))
	Ra.append(0.7)
	Rz.append(0.55)


with open('_slash_collision_avoidance_slash_status.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	status.append(int(row[1]))
status.append(0)	


fig = plt.figure()
ax = fig.add_subplot(211)
ax.plot(dtime,d1,'r', label='Dystans L-F1')
ax.plot(dtime,d2,'g', label='Dystans L-F2')
ax.plot(dtime,d3,'b', label='Dystans F1-F2')
ax.plot(dtime,Rz,'k',linewidth=0.3)
ax.plot(dtime,Ra,'k',linewidth=0.3)
ax.fill_between(dtime, Rz, Ra, color='yellow', alpha=0.5, label='Strefa alarmowa')
ax.fill_between(dtime, 0, Rz, color='orange', alpha=0.5, label='Strefa zabroniona')
ax.set_ylabel('d [m]')
ax.set_xlabel('czas [s]')
scale_x = 1e9
ticks_x = tick.FuncFormatter(lambda dtime, pos: '{0:g}'.format(dtime/scale_x))
ax.xaxis.set_major_formatter(ticks_x)
ax.legend(loc='upper left')
ax.grid(True)

ax1 = fig.add_subplot(212,sharex=ax)
ax1.plot(dtime,status,'m', label='Status kolizji')
ax1.set_ylabel('status')
ax1.set_xlabel('czas [s]')
scale_x = 1e9
ticks_x = tick.FuncFormatter(lambda dtime, pos: '{0:g}'.format(dtime/scale_x))
ax1.xaxis.set_major_formatter(ticks_x)
ax1.legend()
ax1.grid(True)


plt.show()
