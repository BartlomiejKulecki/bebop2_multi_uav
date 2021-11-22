import matplotlib.pyplot as plt
import csv

time = []
target = []
actual = []


with open('_slash_position_controller_data.csv','r') as csvfile:
   next(csvfile)
   plots = csv.reader(csvfile, delimiter=',')
   for row in plots:
        time.append(int(row[0]))
        target.append(float(row[2]))
	actual.append(float(row[7]))
	

plt.plot(time,actual,'r', label='actual')
plt.plot(time,target,'b', label='target')
plt.ylabel('x')
plt.legend()
plt.grid(True)

plt.show()
