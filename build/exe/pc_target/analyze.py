import numpy as np
from pyquaternion import Quaternion
import math
import csv



def euler2quat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp
    return Quaternion([w,x,y,z])

dt = 0.5 #s
with open("output.csv","w") as csvfile:
	outs = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
	outs.writerow(['% Position error', 'Rotation error (deg)', 'Rotation rate error (deg/s)', 'Velocity (m/s)', 'Distance (m)'])
	for i in range(1,99):
		direct = "test" + str(i)
		pos_calc = np.loadtxt(direct + "/position_act.txt")
		pos = np.loadtxt(direct + "/position.txt")
		rot_calc = np.loadtxt(direct + "/rotation_act.txt")
		rot = np.loadtxt(direct + "/rotation.txt")
		num = pos_calc.shape[0]

		trans_vel = np.zeros(num)
		for j in range(num):
			q_calc = Quaternion(rot_calc[j,0], rot_calc[j,1], rot_calc[j,2], rot_calc[j,3])
			q = euler2quat(rot[j,0], rot[j,1], rot[j,2])
			t_calc = q_calc.rotate(pos_calc[j,:])
			t = pos[j,:]
			t[2] = 10 - t[2] 
			dist = np.linalg.norm(t_calc)
			t_error = np.linalg.norm(t - t_calc) / dist
			qdiff = q_calc * q.inverse
			q_error = abs(qdiff.degrees)
			if j > 0:
				roll_calc = ((q_calc * q_calc_old.inverse).degrees)/dt
				roll = ((q*q_old.inverse).degrees)/dt
				ang_Vel = abs(roll - roll_calc)
				trans_vel = np.linalg.norm(t_calc - t_old)/dt
			else:
				ang_Vel = 0
				trans_vel = 0
			q_calc_old = Quaternion(q_calc)
			q_old = Quaternion(q)
			t_old = np.copy(t_calc)
			outs.writerow([str(t_error), str(q_error), str(ang_Vel), str(trans_vel), str(dist)])

