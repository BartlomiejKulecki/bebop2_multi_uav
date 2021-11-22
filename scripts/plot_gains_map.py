import matplotlib.pyplot as plt
import csv
import matplotlib.ticker as tick
import numpy as np

kp = []
kd = []
J = []
tmp=0
kp_prev = 0
kd_prev = 0
J_prev = 0

with open('_slash_gain_kp.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	if kp_prev!=float(row[1]):
        	kp.append( round(float(row[1]),3))
	kp_prev = float(row[1])

with open('_slash_gain_kd.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	if tmp==0:
		first_kd = float(row[1])
		tmp=1
	if kd_prev!=float(row[1]):
		if (float(row[1])==first_kd) & (tmp==2):
			break
        	kd.append(round(float(row[1]),3))
	kd_prev = float(row[1])
	tmp=2

with open('_slash_perf.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
	if J_prev > float(row[1]):
        	J.append(J_prev)
	J_prev = float(row[1])

m = len(kp)
n = len(kd)
xtick = np.arange(0,m)
ytick = np.arange(0,n)

Jmat = np.mat(J)
Jmat= Jmat.reshape(m,m)

fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow( Jmat ,interpolation='nearest', origin='lower',  cmap='viridis')
ax.set_ylabel('Kd', fontsize=16)
ax.set_xlabel('Kp', fontsize=16)
plt.xticks(xtick, kp)
plt.yticks(ytick, kd)
cb = fig.colorbar(im)
cb.set_label(label='J value',size=16)

plt.show()
