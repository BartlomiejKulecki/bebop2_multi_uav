import matplotlib.pyplot as plt
import csv
import matplotlib.ticker as tick

t = []
target = []
actual = []
tmp=0

with open('_slash_position_controller_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	if tmp==0:
		zero = int(row[0])
		tmp = 1
        t.append( (int(row[0])-zero) )
        target.append(float(row[3]))
	actual.append(float(row[8]))
	

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t,actual,'r', label='Actual pose')
ax.plot(t,target,'b', label='Reference')
ax.set_ylabel('y [m]')
ax.set_xlabel('time [s]')
scale_x = 1e9
ticks_x = tick.FuncFormatter(lambda t, pos: '{0:g}'.format(t/scale_x))
ax.xaxis.set_major_formatter(ticks_x)

ax.legend()
ax.grid(True)

plt.show()
