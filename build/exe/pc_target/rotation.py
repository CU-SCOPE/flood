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
with open("rotation.csv","w") as csvfile:
	outs = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
	outs.writerow(['Quat_calc w', 'Quat_calc x', 'Quat_calc y', 'Quat_calc z', 
		           'Quat_meas w', 'Quat_meas x', 'Quat_meas y', 'Quat_meas z', 
		           'Rate_calc w', 'Rate_calc x', 'Rate_calc y', 'Rate_calc z',
		           'Rate_meas w', 'Rate_meas x', 'Rate_meas y', 'Rate_meas z',
		           'Distance (m)', 'Time'])
	for i in range(1,99):
		direct = "test" + str(i)
		pos_calc = np.loadtxt(direct + "/position_act.txt")
		pos = np.loadtxt(direct + "/position.txt")
		rot_calc = np.loadtxt(direct + "/rotation_act.txt")
		rot = np.loadtxt(direct + "/rotation.txt")
		num = pos_calc.shape[0]

		trans_vel = np.zeros(num)
		time = 0
		for j in range(num):
			q_calc = Quaternion(rot_calc[j,0], rot_calc[j,1], rot_calc[j,2], rot_calc[j,3])
			q = euler2quat(rot[j,0], rot[j,1], rot[j,2])
			t_calc = q_calc.rotate(pos_calc[j,:])
			t = pos[j,:]
			t[2] = 10 - t[2] 
			dist = np.linalg.norm(t_calc)
			if j > 0:
				roll_calc = ((q_calc * q_calc_old.inverse))/dt
				roll = ((q*q_old.inverse))/dt
			else:
				roll_calc = [0]*4
				roll = [0]*4
			q_calc_old = Quaternion(q_calc)
			q_old = Quaternion(q)
			t_old = np.copy(t_calc)
			outs.writerow([str(q_calc[0]), str(q_calc[1]), str(q_calc[2]), str(q_calc[3]),
						   str(q[0]), str(q[1]), str(q[2]), str(q[3]),
						   str(roll_calc[0]), str(roll_calc[1]), str(roll_calc[2]), str(roll_calc[3]),
						   str(roll[0]), str(roll[1]), str(roll[2]), str(roll[3]),
			 			   str(dist), str(time)])
			time += 0.5

