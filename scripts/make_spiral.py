import csv
import math as m

x = []
y = []
z = []
yaw = []

r = 1.0
kat = 0.0
h = 1.0
yaw_angle = -180.0
okr=0

x.append(r)
y.append(0.0)
z.append(h)
yaw.append(yaw_angle)

while okr < 4:
	kat = kat+10.0
	if yaw_angle==180:
		yaw_angle=-180.0
	yaw_angle = yaw_angle+10.0
	h = h+0.03
	x.append(r*m.cos(kat*m.pi/180.0))
	y.append(r*m.sin(kat*m.pi/180.0))
	z.append(h)
	yaw.append(yaw_angle)
	if yaw_angle==180:
		okr=okr+1

it = 0

with open('spirala.csv', mode='w') as csvfile:
    spirala_writer = csv.writer(csvfile, delimiter=',')
    while it < len(x):
    	spirala_writer.writerow([ x[it], y[it], z[it], yaw[it]])
	it=it+1
    
