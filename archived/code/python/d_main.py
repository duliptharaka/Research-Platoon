from __future__ import division
import os
import numpy as np
import scipy.integrate as integrate
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import time
import bigfloat
import pickle

# Read the pre-defined coefficients of the cost functions
def read_coff():
	w1 = 4*(10**7)
	w3 = 10
	w4 = 200
	w5 = 0.5
	r = 3
	alpha = 0.2
	R = 30
	h_d = 3
	return w1, w3, w4, w5, r, alpha, R, h_d

# Change unit
def change_unit( s,v,a,fuel, delta_t ):
	# Change metric to American unit
	# Convert unit
	# 1 m = 0.000621371 mile
	# 1 miles/h2 = 0.00012417777777778 m/s2
	# 1 mph = 0.447 04 m/s
	# 1 mL/s = 0.951019 gallon/hour
	# 1L = 0.264172 gallon

	# Convert s from meter to mile
	# Convert v from meter/second to mile/hour (mph)
	[m,n] = s.shape
	for i in range(m):
		for j in range(n):   
		    s[i,j] = s[i,j] * 0.000621371
		    v[i,j] = v[i,j]/0.44704
	# Convert fuel from kg/s to mile/gallon
	# gasoline density is 0.75 Kg/L
	[m,n] = fuel.shape
	for i in range(m):
		for j in range(n):   
		    fuel[i,j] = (s[i+1,j]- s[i,j])/((fuel[i,j]*1000/0.75)*delta_t/1000*0.264172)
	return s,v,a,fuel

def my_exp(x):
	result = 1 + x + x*x/2 + x**3/6 + x**4/24
	return result

# Function for middle car
#def d_middle(x, para, v, v_front, v_behind, d_front, TotalS, TotalF, delta_t):
def d_middle(x, v, v_front, v_behind, d_front, TotalS, TotalF, delta_t):
	a = x[0]
	a_front = x[1]
	a_behind = x[2]
	s = v*delta_t + (a/2)*(delta_t**2)
	v_next = v + a*delta_t
	v_front_next = v_front + a_front*delta_t
	v_behind_next = v_behind + a_behind*delta_t
	d_front_next = d_front + (v_front-v)*delta_t + ((delta_t**2)/2)*(a_front-a)
	w1, w3, w4, w5, r, alpha, R, h_d = read_coff()
	#w2 = r * bigfloat.exp(-alpha*d_front_next,bigfloat.precision(100))
	try:
		w2  = r * np.exp(-alpha*d_front_next)
	except:
		w2 = inf
	R_e = R + h_d*v_next - d_front_next
	#w6 = r * bigfloat.exp(-alpha*(2*R-d_front_next), bigfloat.precision(100))/100
	try:
		w6 = r * np.exp(-alpha*(2*R-d_front_next))/100
	except:
		w6 = inf
	R_b = R + h_d*v_next - (2*R-d_front_next)
	# Fuel consumption
	m1 = 1.442*(10**(-6))
	m2 = -5.67*(10**(-6))
	m3 = 1.166*(10**(-6))
	m4 = 39.269*(10**(-6))
	m5 = 58.284*(10**(-6))
	m6 = 19.279*(10**(-6))
	m7 = 82.426*(10**(-6))
	m8 = 185.36*(10**(-6))

	fuel = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
	#target = para[0]*fuel/s + para[1] * np.exp(para(9)*d_front_next)*(R_e**2) + para[2]*((v_next - v_front_next)**2)+ para[3]*(a**2)  + para[4]*((v_next - v_behind_next)**2) + para[5]*((d_front_next+700)**0.5) + para[6] * (d_front_next+700) + para[7] * ((d_front_next+700)**2) + para[9]*abs(v_front_next-v_next) 
	target = w1*fuel/s + w2*(R_e**2) + w4*(a**2)  + 200*((d_front_next+700)**0.5)+5*((d_front_next+700)**2)+ w3*((v_next - v_front_next)**2)+ 1000*np.absolute(v_front_next-v_next)+ w5*((v_next - v_behind_next)**2)
	return target

# Calculate cost function value from the optimized result (middle)
def d_cost_middle(x, v, v_front, v_behind, d_front, TotalS, TotalF, delta_t):
	a = x[0]
	a_front = x[1]
	a_behind = x[2]
	s = v*delta_t + (a/2)*(delta_t**2)
	v_next = v + a*delta_t
	v_front_next = v_front + a_front*delta_t
	v_behind_next = v_behind + a_behind*delta_t
	d_front_next = d_front + (v_front-v)*delta_t + ((delta_t**2)/2)*(a_front-a)
	w1, w3, w4, w5, r, alpha, R, h_d = read_coff() 
	#w2 = r * bigfloat.exp(-alpha*d_front_next, bigfloat.precision(100))
	try:
		w2  = r * np.exp(-alpha*d_front_next)
	except:
		w2 = inf
	R_e = R + h_d*v_next - d_front_next
	#w6 = r * bigfloat.exp(-alpha*(2*R-d_front_next),bigfloat.precision(100))/100
	try:
		w6 = r * np.exp(-alpha*(2*R-d_front_next))/100
	except:
		w6 = inf
	R_b = R + h_d*v_next - (2*R-d_front_next)
	# Fuel consumption
	m1 = 1.442*(10**(-6))
	m2 = -5.67*(10**(-6))
	m3 = 1.166*(10**(-6))
	m4 = 39.269*(10**(-6))
	m5 = 58.284*(10**(-6))
	m6 = 19.279*(10**(-6))
	m7 = 82.426*(10**(-6))
	m8 = 185.36*(10**(-6))

	fuel = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
	#target = para[0]*fuel/s + para[1] * np.exp(para(9)*d_front_next)*(R_e**2) + para[2]*((v_next - v_front_next)**2)+ para[3]*(a**2)  + para[4]*((v_next - v_behind_next)**2) + para[5]*((d_front_next+700)**0.5) + para[6] * (d_front_next+700) + para[7] * ((d_front_next+700)**2) + para[9]*abs(v_front_next-v_next) 
	#target = w1*fuel/s + w2*(R_e**2) + w4*(a**2)  + 200*((d_front_next+700)**0.5)+5*((d_front_next+700)**2)+ w3*((v_next - v_front_next)**2)+ 1000*abs(v_front_next-v_next)+ w5*((v_next - v_behind_next)**2)
	p1 = w1*fuel/s
	p2 = w2*(R_e**2) + 200*((d_front_next+700)**0.5)+5*((d_front_next+700)**2)
	p3 = w3*((v_next - v_front_next)**2)+ 1000*abs(v_front_next-v_next)
	p4 = w4*a**2
	p5 = w5*((v_next - v_behind_next)**2)
	all_p = p1 + p2 + p3 + p4 + p5
	return p1, p2, p3, p4, p5, fuel, all_p

# Function for last car
def d_last(x, v, v_front, d_front, TotalS, TotalF, delta_t):
	a = x[0]
	a_front = x[1]
	s = v*delta_t + (a/2)*delta_t**2
	v_next = v + a*delta_t
	v_front_next = v_front + a_front*delta_t
	d_front_next = d_front + (v_front-v)*delta_t + (delta_t**2/2)*(a_front-a)
	w1, w3, w4, w5, r, alpha, R, h_d = read_coff()
	#w2 = r * bigfloat.exp(-alpha*d_front_next,bigfloat.precision(100))
	try:
		w2  = r * np.exp(-alpha*d_front_next)
	except:
		w2 = inf
	R_e = R + h_d*v_next - d_front_next
	#w6 = r * bigfloat.exp(-alpha*(2*R-d_front_next),bigfloat.precision(100))/100
	try:
		w6 = r * np.exp(-alpha*(2*R-d_front_next))/100
	except:
		w6 = inf
	R_b = R + h_d*v_next - (2*R-d_front_next)
	# Fuel consumption
	m1 = 1.442*(10**(-6))
	m2 = -5.67*(10**(-6))
	m3 = 1.166*(10**(-6))
	m4 = 39.269*(10**(-6))
	m5 = 58.284*(10**(-6))
	m6 = 19.279*(10**(-6))
	m7 = 82.426*(10**(-6))
	m8 = 185.36*(10**(-6))
	fuel = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
	target = w1*fuel/s + w2*(R_e**2) + 200*((d_front_next+700)**0.5)+5*((d_front_next+700)**2)+ w4*(a**2) + w3*((v_next - v_front_next)**2)+ 1000*np.absolute(v_front_next-v_next)
	return target

# Calculate cost function value from the optimized result (last)
def d_cost_last(x, v, v_front, d_front, TotalS, TotalF, delta_t):
	a = x[0]
	a_front = x[1]
	s = v*delta_t + (a/2)*delta_t**2
	v_next = v + a*delta_t
	v_front_next = v_front + a_front*delta_t
	d_front_next = d_front + (v_front-v)*delta_t + (delta_t**2/2)*(a_front-a)
	[ w1, w3, w4, w5, r, alpha, R, h_d ] = read_coff()
	#w2 = r * bigfloat.exp(-alpha*d_front_next,bigfloat.precision(100))
	try:
		w2  = r * np.exp(-alpha*d_front_next)
	except:
		w2 = inf
	R_e = R + h_d*v_next - d_front_next
	#w6 = r * bigfloat.exp(-alpha*(2*R-d_front_next),bigfloat.precision(100))/100
	try:
		w6 = r * np.exp(-alpha*(2*R-d_front_next))/100
	except:
		w6 = inf
	R_b = R + h_d*v_next - (2*R-d_front_next)

	m1 = 1.442*(10**(-6))
	m2 = -5.67*(10**(-6))
	m3 = 1.166*(10**(-6))
	m4 = 39.269*(10**(-6))
	m5 = 58.284*(10**(-6))
	m6 = 19.279*(10**(-6))
	m7 = 82.426*(10**(-6))
	m8 = 185.36*(10**(-6))
	fuel = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
	p1 = w1*fuel/s
	p2 = w2*(R_e**2) + 200*((d_front_next+700)**0.5)+5*((d_front_next+700)**2)
	p3 = w3*((v_next - v_front_next)**2)+ 1000*np.absolute(v_front_next-v_next)
	p4 = w4*a**2
	all_p = p1 + p2 + p3 + p4
	return p1, p2, p3, p4,fuel, all_p

#----------------------------------------------------------------------------
# Main function
if __name__ == '__main__':
	start = time.time()
	current_dir = os.path.dirname(os.path.realpath(__file__))
	# Load first car driving patterns
	f = open('first_car_v')
	line = f.readlines()
	f.close()
	first_car_v = [float(line[i].strip()) for i in range(len(line))]
	f = open('first_car_a')
	line = f.readlines()
	f.close()
	first_car_a = [float(line[i].strip()) for i in range(len(line))]
	# Number of total vehicles in the platooning system
	number = 9
	# update interval is dt = 1 second
	dt = 1
	# total time considered is 700 seconds, limited by the first car driving pattern
	step = 699
	# max/min velocity and acceleration allowed, based on the first driving pattern
	v_min = 28*0.44704 # 12.5
	v_max = 60*0.44704 # 26.8
	a_min = -1
	a_max = 1
	# Coefficients of the fuel consumption
	m1 = 1.442*(10**(-6))
	m2 = -5.67*(10**(-6))
	m3 = 1.166*(10**(-6))
	m4 = 39.269*(10**(-6))
	m5 = 58.284*(10**(-6))
	m6 = 19.279*(10**(-6))
	m7 = 82.426*(10**(-6))
	m8 = 185.36*(10**(-6))
	# Kalman filter paramaters
	middle_A = np.array([[1,dt,0,0,0,0],[0,1,0,0,0,0],[0,-dt,1,0,dt,0],[0,0,0,1,dt,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
	middle_B = np.array([[0,dt**2/2,0], [0,dt,0],[dt**2/2,-dt**2/2,0],[dt**2/2,0,0],[dt,0,0],[0,0,dt]])
	middle_C = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[0,0,0,0,0,1]])
	middle_start_P = np.zeros((6,6))
	last_A = np.array([[1,dt,0,0,0],[0,1,0,0,0],[0,-dt,1,0,dt],[0,0,0,1,dt],[0,0,0,0,1]])
	last_B = np.array([[0,dt**2/2],[0,dt],[dt**2/2,-dt**2/2],[dt**2/2,0], [dt,0]])
	last_C = np.array([[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[0,0,0,1,0],[0,0,0,0,1],[0,0,0,0,1]])
	last_start_P = np.zeros((5,5))
	# Initiate matrix and varibles
	output_v = np.zeros((step+1, number))
	output_s = np.zeros((step+1, number))
	output_a = np.zeros((step+1, number))
	output_fuel = np.zeros((step, number))
	output_cost = np.zeros((step, number-1, 5))
	# Initiate car position so that
	## all cars are 40 meters (0.0248548 miles) apart
	## all cars start at the same veolocity/acceleration as the first car
	start_s = np.zeros((number))
	start_v = np.zeros((number))
	start_a = np.zeros((number))
	# Record the cumulative distance and fuel consumption
	TotalS = np.zeros((number-1))
	TotalF = np.zeros((number-1))
	for j in range(1, number+1):
		start_s[j-1] = 40*(number-1) - (j-1)*40
		start_v[j-1] = first_car_v[0]
		start_a[j-1] = first_car_a[0]
	# Generate position for the first car
	first_car_s = np.zeros((700))
	first_car_s[0] = start_s[0]
	for i in range(1, step+1):
		first_car_s[i] = first_car_s[i-1] + first_car_v[i-1]*dt + (first_car_a[i-1]/2)*(dt**2)
	# Calculate fuel consumption for the first car
	first_car_fuel = np.zeros((step))
	for j in range(step):
		v = first_car_v[j]
		a = first_car_a[j]
		first_car_fuel[j] = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
	output_fuel[:,0] = first_car_fuel
	output_v[0,:] = start_v
	output_s[0,:] = start_s
	output_a[0,:] = start_a   
	# Start the main function
	# Do step = 1 separately because it is a little different for KF
	i = 0
	# Optimization for the middle car (2,3,4th car in a 5-car model)
	middle_state_last = np.zeros((6, step, number-2))
	middle_control_last = np.zeros((3, 1, number-2))
	middle_pred = np.zeros((6, 1, number-2))
	middle_P_last = np.zeros((6, 6, number-2))
	middle_state = np.zeros((6, step, number-2))
	middle_obs = np.zeros((8, step, number-2))    
	middle_est = np.zeros((6, 1, number-2))
	middle_P = np.zeros((6, 6, number-2))
	middle_K_F = np.zeros((6, 8, number-2))
	last_state_last = np.zeros((5, step))
	last_state = np.zeros((5, step))
	last_obs = np.zeros((6, step))
	start_s_new = np.zeros((number))
	start_v_new = np.zeros((number))
	start_a_new = np.zeros((number))    
	for j in range(1, number-1):
		# initial state variable
		middle_state_last[:,0,j-1] = np.array([start_s[j],start_v[j],start_s[j-1]-start_s[j],start_s[j-1],start_v[j-1],start_v[j+1]])
		middle_control_last[:,:,j-1] = np.array([[start_a[j-1]],[start_a[j]],[start_a[j+1]]])
		# calculate state updates (add plant noise)
		middle_state[:,0,j-1] = (np.dot(middle_A, np.transpose([middle_state_last[:,0,j-1]])) + np.dot(middle_B, middle_control_last[:,:,j-1])).flatten()
		# get the observation (add sensor noise)
		middle_obs[:,0,j-1] = (np.dot(middle_C, np.transpose([middle_state[:,0,j-1]]))).flatten()
		# calculate cumulative distance and fuel
		TotalS[j-1] = middle_obs[0,0,j-1] - start_s[j]
		v = start_v[j]
		a = start_a[j]
		TotalF[j-1] = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
		# minimize cost function
		v_front = (middle_obs[4,0,j-1] + middle_obs[5,0,j-1])/2
		v_behind = (middle_obs[6,0,j-1] + middle_obs[7,0,j-1])/2
		output_fuelront = (middle_obs[2,0,j-1] + (middle_obs[3,0,j-1] - middle_obs[0,0,j-1]))/2
		v = middle_obs[1,0,j-1]
		s = middle_obs[0,0,j-1]
		# optimization parameter
		# Use the PyIPopt (nonlinear programming solver) from https://github.com/xuy/pyipopt
		# Use Scipy (SLSQP) from https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.optimize.minimize.html
		# example in http://www.bioinfo.org.cn/~wangchao/maa/Numerical_Optimization.pdf page 462

		#g = @(x)d_middle(x, para, v, v_front, v_behind, output_fuelront, TotalS(j-1), TotalF(j-1), dt)
		#lb = [max(a_min,(v_min-v)/dt), max(a_min,(v_min-v_front)/dt), max(a_min,(v_min-v_behind)/dt)]
		#ub = [min(a_max,(v_max-v)/dt), min(a_max,(v_max-v_front)/dt),min(a_max,(v_max-v_behind)/dt)]
		#x = fmincon(g, x0, A, b, Aeq, beq, lb, ub, [], options)
		bnds = ((max(a_min,(v_min-v)/dt), min(a_max,(v_max-v)/dt)),(max(a_min,(v_min-v_front)/dt), min(a_max,(v_max-v_front)/dt)),(max(a_min,(v_min-v_behind)/dt), min(a_max,(v_max-v_behind)/dt)))
		A = np.array([[1,-1,0],[-1,0,1]])
		b = np.array([[2*(start_s[j-1]-start_s[j]+(v_front-v)*dt)/(dt**2)],[2*(start_s[j]-start_s[j+1]+(v-v_behind)*dt)/(dt**2)]])
		cons = ({'type': 'ineq', 'fun': lambda x:  b[0] - A[0,0]*x[0] - A[0,1]*x[1] - A[0,2]*x[2]}, {'type': 'ineq', 'fun': lambda x:  b[1] - A[1,0]*x[0] - A[1,1]*x[1] - A[1,2]*x[2]})
		res = minimize(d_middle, (0,0,0) , args=(v, v_front, v_behind, output_fuelront, TotalS[j-1], TotalF[j-1], dt), method='SLSQP', bounds=bnds, constraints=cons)
		x = res.x
		# Retrieve each element of cost function
		[p1,p2,p3,p4,p5,one_fuel, all_p] = d_cost_middle(x, v, v_front, v_behind, output_fuelront, TotalS[j-1], TotalF[j-1], dt)
		# retrieve result
		output_fuel[i,j] = one_fuel
		output_cost[i,j-1,0] = p1
		output_cost[i,j-1,1] = p2       
		output_cost[i,j-1,2] = p3
		output_cost[i,j-1,3] = p4  
		output_cost[i,j-1,4] = p5  
		start_s_new[j] = s
		start_v_new[j] = v
		start_a_new[j] = x[0]

	# For the last car
	j = j+1
	# initial state variable
	last_state_last[:,0] = np.array([start_s[j], start_v[j], start_s[j-1]-start_s[j], start_s[j-1], start_v[j-1]])
	last_control_last = np.array([[start_a[j-1]], [start_a[j]]])
	# calculate state updates (add plant noise)
	last_state[:,0] = (np.dot(last_A, last_state_last[:,0].reshape(5,1)) + np.dot(last_B, last_control_last)).flatten()
	# get the observation (add sensor noise)
	last_obs[:,0] = (np.dot(last_C, last_state[:,0].reshape(5,1))).flatten()
	# calculate cumulative distance and fuel
	TotalS[j-1] = last_obs[0,0] - start_s[j]
	v = start_v[j]
	a = start_a[j]
	TotalF[j-1] = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
	# Minimize cost function
	v_front = (last_obs[4,0] + last_obs[5,0])/2
	output_fuelront = (last_obs[2,0] + (last_obs[3,0]-last_obs[0,0]))/2
	v = last_obs[1,0]
	s = last_obs[0,0]

	# optimization parameter
	#h = @(x)d_last(x, para, v, v_front, output_fuelront, TotalS(j-1), TotalF(j-1), dt)
	#lb = [max(a_min,((v_min-v)/dt)), max(a_min,((v_min-v_front)/dt))]
	#ub = [min(a_max,((v_max-v)/dt)), min(a_max,((v_max-v_front)/dt))]
	#A = [1,-1]
	#b = [2*(start_s(j-1)-start_s(j)+(v_front-v)*dt)/(dt**2)]
	#Aeq = []
	#beq = []
	#x0 = [00]
	#x = fmincon(h, x0, A, b, Aeq, beq, lb, ub, [], options)
	bnds = ((max(a_min,((v_min-v)/dt)), min(a_max,((v_max-v)/dt))),(max(a_min,((v_min-v_front)/dt)),min(a_max,((v_max-v_front)/dt))))
	A = np.array([1,-1])
	b = np.array([2*(start_s[j-1]-start_s[j]+(v_front-v)*dt)/(dt**2)])
	cons = ({'type': 'ineq', 'fun': lambda x:  b[0] - A[0]*x[0] - A[1]*x[1]})
	res = minimize(d_last, (0,0) , args=(v, v_front, output_fuelront, TotalS[j-1], TotalF[j-1], dt), method='SLSQP', bounds=bnds, constraints=cons)
	x = res.x
	# Retrieve each element of cost function
	p1,p2,p3,p4,one_fuel, all_p = d_cost_last(x, v, v_front, output_fuelront, TotalS[j-1], TotalF[j-1], dt)
	output_fuel[i,j] = one_fuel
	output_cost[i,j-1,0] = p1
	output_cost[i,j-1,1] = p2        
	output_cost[i,j-1,2] = p3
	output_cost[i,j-1,3] = p4
	start_s_new[j] = s
	start_v_new[j] = v
	start_a_new[j] = x[0]   
	# For the first car
	start_s_new[0] = first_car_s[1]
	start_v_new[0] = first_car_v[1]
	start_a_new[0] = first_car_a[1]
	# update parameter
	start_s = start_s_new
	start_v = start_v_new
	start_a = start_a_new
	output_v[i+1,:] = start_v
	output_s[i+1,:] = start_s
	output_a[i+1,:] = start_a

	
	#---------------------------------------------------------------------------
	# For step from 2, KF is a little bit different
	for i in range(1, step):
		print '-'*20
		print i
		start_s_new = np.zeros((number))
		start_v_new = np.zeros((number))
		start_a_new = np.zeros((number)) 
		start_s_new[0] = first_car_s[i+1]
		start_v_new[0] = first_car_v[i+1] 
		start_a_new[0] = first_car_a[i+1]   
		for j in range(1, number-1):
			# # initial state variable (add one column to the existing state variable)
			middle_state_last[:, i, j-1]= np.array([start_s[j], start_v[j], start_s[j-1]-start_s[j],start_s[j-1],start_v[j-1],start_v[j+1]])
			middle_control_last[:,:,j-1] = np.array([[start_a[j-1]],[start_a[j]],[start_a[j+1]]])
			# # calculate state updates (add plant noise)
			middle_state[:,i,j-1] = (np.dot(middle_A, middle_state_last[:,i,j-1].reshape(6,1)) + np.dot(middle_B, middle_control_last[:,:,j-1])).flatten()
			# # get the observation (add sensor noise)
			middle_obs[:,i,j-1] = np.dot(middle_C, middle_state[:,i,j-1])
			# calculate cumulative distance and fuel
			TotalS[j-1] = TotalS[j-1] + middle_obs[1,i,j-1] - middle_obs[1,i-1,j-1]
			v = middle_obs[2,i-1,j-1]
			a = start_a[j]
			TotalF[j-1] = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
			# # minimize cost function
			v_front = (middle_obs[4,i,j-1] + middle_obs[5,i,j-1])/2
			v_behind = (middle_obs[6,i,j-1] + middle_obs[7,i,j-1])/2
			output_fuelront = (middle_obs[2,i,j-1] + (middle_obs[3,i,j-1] - middle_obs[0,i,j-1]))/2
			v = middle_obs[1,i,j-1]
			s = middle_obs[0,i,j-1]

			# optimization parameter
			#g = @(x)d_middle(x, para, v, v_front, v_behind, output_fuelront, TotalS(j-1), TotalF(j-1), dt)
			#lb = [max(a_min,(v_min-v)/dt), max(a_min,(v_min-v_front)/dt), max(a_min,(v_min-v_behind)/dt)]
			#ub = [min(a_max,(v_max-v)/dt), min(a_max,(v_max-v_front)/dt),min(a_max,(v_max-v_behind)/dt)]
			#A = [1,-1,0 -1,0,1]
			#b = [2*(start_s(j-1)-start_s(j)+(v_front-v)*dt)/(dt**2)2*(start_s(j)-start_s(j+1)+(v-v_behind)*dt)/(dt**2)]
			#Aeq = []
			#beq = []
			#x0 = [000]
			#x = fmincon(g, x0, A, b, Aeq, beq, lb, ub, [], options)
			bnds = ((max(a_min,(v_min-v)/dt), min(a_max,(v_max-v)/dt)),(max(a_min,(v_min-v_front)/dt), min(a_max,(v_max-v_front)/dt)),(max(a_min,(v_min-v_behind)/dt), min(a_max,(v_max-v_behind)/dt)))
			A = np.array([[1,-1,0],[-1,0,1]])
			b = np.array([[2*(start_s[j-1]-start_s[j]+(v_front-v)*dt)/(dt**2)],[2*(start_s[j]-start_s[j+1]+(v-v_behind)*dt)/(dt**2)]])
			cons = ({'type': 'ineq', 'fun': lambda x:  b[0] - A[0,0]*x[0] - A[0,1]*x[1] - A[0,2]*x[2]}, {'type': 'ineq', 'fun': lambda x:  b[1] - A[1,0]*x[0] - A[1,1]*x[1] - A[1,2]*x[2]})
			res = minimize(d_middle, (0,0,0) , args=(v, v_front, v_behind, output_fuelront, TotalS[j-1], TotalF[j-1], dt), method='SLSQP', bounds=bnds, constraints=cons)
			x = res.x
			# Retrieve each element of cost function
			p1,p2,p3,p4,p5,one_fuel, all_p = d_cost_middle(x, v, v_front, v_behind, output_fuelront, TotalS[j-1], TotalF[j-1], dt)
			# retrieve result
			output_fuel[i,j] = one_fuel
			output_cost[i,j-1,0] = p1
			output_cost[i,j-1,1] = p2       
			output_cost[i,j-1,2] = p3
			output_cost[i,j-1,3] = p4  
			output_cost[i,j-1,4] = p5  
			start_s_new[j] = s
			start_v_new[j] = v
			start_a_new[j] = x[0]

		# For the last car
		j = j+1
		# # initial state variable (add one column)
		last_state_last[:,i] = np.array([start_s[j], start_v[j], start_s[j-1]-start_s[j], start_s[j-1], start_v[j-1]])
		last_control_last = np.array([[start_a[j-1]], [start_a[j]]])
		# # calculate state updates (add plant noise)
		last_state[:,i] = (np.dot(last_A, last_state_last[:,i].reshape(5,1)) + np.dot(last_B, last_control_last)).flatten()
		# # get the observation (add sensor noise)
		last_obs[:,i] = (np.dot(last_C, last_state[:,i].reshape(5,1))).flatten()
		# calculate cumulative distance and fuel
		TotalS[j-1] = TotalS[j-1] + last_obs[0,i] - last_obs[0,i-1]
		v = last_obs[1,i-1]
		a = start_a[j]
		TotalF[j-1] = integrate.quad(lambda x: m1*((v+a*x)**2)+m2*(a**2)+m3*((v+a*x)**2)*a + m4*(v+a*x)*(a**2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8, 0, 1)[0]
		# # Minimize cost function
		v_front = (last_obs[4,i] + last_obs[5,i])/2
		output_fuelront = (last_obs[2,i] + (last_obs[3,i]-last_obs[0,i]))/2
		v = last_obs[1,i]
		s = last_obs[0,i]

		# optimization parameter
		bnds = ((max(a_min,((v_min-v)/dt)), min(a_max,((v_max-v)/dt))),(max(a_min,((v_min-v_front)/dt)),min(a_max,((v_max-v_front)/dt))))
		A = np.array([1,-1])
		b = np.array([2*(start_s[j-1]-start_s[j]+(v_front-v)*dt)/(dt**2)])
		cons = ({'type': 'ineq', 'fun': lambda x:  b[0] - A[0]*x[0] - A[1]*x[1]})
		res = minimize(d_last, (0,0) , args=(v, v_front, output_fuelront, TotalS[j-1], TotalF[j-1], dt), method='SLSQP', bounds=bnds, constraints=cons)
		x = res.x
		# Retrieve each element of cost function
		p1,p2,p3,p4,one_fuel, all_p = d_cost_last(x, v, v_front, output_fuelront, TotalS[j-1], TotalF[j-1], dt)
		# Retrieve each element of cost function
		p1,p2,p3,p4,one_fuel, all_p = d_cost_last(x, v, v_front, output_fuelront, TotalS[j-1], TotalF[j-1], dt)
		output_fuel[i,j] = one_fuel
		output_cost[i,j-1,0] = p1
		output_cost[i,j-1,1] = p2        
		output_cost[i,j-1,2] = p3
		output_cost[i,j-1,3] = p4
		start_s_new[j] = s
		start_v_new[j] = v
		start_a_new[j] = x[0]   
		# update parameter
		start_s = start_s_new
		start_v = start_v_new
		start_a = start_a_new
		output_v[i+1,:] = start_v
		output_s[i+1,:] = start_s
		output_a[i+1,:] = start_a
	print time.time()-start
	
	[output_s, output_v, output_a, output_fuel] = change_unit(output_s, output_v, output_a, output_fuel, dt)
	# Save numpy array to file
	np.savetxt('output_s', output_s, delimiter=',')
	np.savetxt('output_v', output_v, delimiter=',')
	np.savetxt('output_a', output_a, delimiter=',')
	np.savetxt('output_fuel', output_fuel, delimiter=',')
	# Save result to pickles
	pickle.dump(output_s, open('output_s.p','wb'))
	pickle.dump(output_v, open('output_v.p','wb'))
	pickle.dump(output_a, open('output_a.p','wb'))
	pickle.dump(output_fuel, open('output_fuel.p','wb'))

	# Plot the result
	#color = ['k','r','b','g','y','m','c', [1,0.5,0], [0.7,0.5,1], [0, 0.5, 0], [0, 0.5, 1], [0.5, 0, 1], [0.5, 0, 0]]; 
	color = ['k','r','b','g','y','m','c', 'crimson','orange']
	time = np.arange(0, step+1, 1)
	plt.figure(1)
	plt.subplot(321)
	plt.plot(time, output_a[:,0], color[0], label = 'car1')
	plt.plot(time, output_a[:,1], color[1], label = 'car2')
	plt.plot(time, output_a[:,2], color[2], label = 'car3')
	plt.plot(time, output_a[:,3], color[3], label = 'car4')
	plt.plot(time, output_a[:,4], color[4], label = 'car5')
	plt.plot(time, output_a[:,5], color[5], label = 'car6')
	plt.plot(time, output_a[:,6], color[6], label = 'car7')
	plt.plot(time, output_a[:,7], color[7], label = 'car8')
	plt.plot(time, output_a[:,8], color[8], label = 'car9')
	plt.title('Acceleration')
	plt.xlabel('Time (s)')
	plt.ylabel('Acceleration (m/s^2)')
	plt.legend()
	#plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)


	plt.subplot(322)
	plt.plot(time, output_v[:,0], color[0], label = 'car1')
	plt.plot(time, output_v[:,1], color[1], label = 'car2')
	plt.plot(time, output_v[:,2], color[2], label = 'car3')
	plt.plot(time, output_v[:,3], color[3], label = 'car4')
	plt.plot(time, output_v[:,4], color[4], label = 'car5')
	plt.plot(time, output_v[:,5], color[5], label = 'car6')
	plt.plot(time, output_v[:,6], color[6], label = 'car7')
	plt.plot(time, output_v[:,7], color[7], label = 'car8')
	plt.plot(time, output_v[:,8], color[8], label = 'car9')
	plt.title('Velocity')
	plt.xlabel('Time (s)')
	plt.ylabel('Velocity (mile/hour)')


	plt.subplot(323)
	plt.plot(time, output_s[:,0], color[0], label = 'car1')
	plt.plot(time, output_s[:,1], color[1], label = 'car2')
	plt.plot(time, output_s[:,2], color[2], label = 'car3')
	plt.plot(time, output_s[:,3], color[3], label = 'car4')
	plt.plot(time, output_s[:,4], color[4], label = 'car5')
	plt.plot(time, output_s[:,5], color[5], label = 'car6')
	plt.plot(time, output_s[:,6], color[6], label = 'car7')
	plt.plot(time, output_s[:,7], color[7], label = 'car8')
	plt.plot(time, output_s[:,8], color[8], label = 'car9')
	plt.title('Distance')
	plt.xlabel('Time (s)')
	plt.ylabel('Distance (mile)')

	plt.subplot(324)
	plt.plot(time, output_s[:,0] - output_s[:,1], color[0], label = 'btw car1 and car 2')
	plt.plot(time, output_s[:,1] - output_s[:,2], color[1], label = 'btw car2 and car 3')
	plt.plot(time, output_s[:,2] - output_s[:,3], color[2], label = 'btw car3 and car 4')
	plt.plot(time, output_s[:,3] - output_s[:,4], color[3], label = 'btw car4 and car 5')
	plt.plot(time, output_s[:,4] - output_s[:,5], color[4], label = 'btw car5 and car 6')
	plt.plot(time, output_s[:,5] - output_s[:,6], color[5], label = 'btw car6 and car 7')
	plt.plot(time, output_s[:,6] - output_s[:,7], color[6], label = 'btw car7 and car 8')
	plt.plot(time, output_s[:,7] - output_s[:,8], color[7], label = 'btw car8 and car 9')
	plt.title('Distance')
	plt.xlabel('Time (s)')
	plt.ylabel('Distance (mile)')

	plt.subplot(325)
	plt.plot(time[:step], output_fuel[:,0], color[0], label = 'car1')
	plt.plot(time[:step], output_fuel[:,1], color[1], label = 'car2')
	plt.plot(time[:step], output_fuel[:,2], color[2], label = 'car3')
	plt.plot(time[:step], output_fuel[:,3], color[3], label = 'car4')
	plt.plot(time[:step], output_fuel[:,4], color[4], label = 'car5')
	plt.plot(time[:step], output_fuel[:,5], color[5], label = 'car6')
	plt.plot(time[:step], output_fuel[:,6], color[6], label = 'car7')
	plt.plot(time[:step], output_fuel[:,7], color[7], label = 'car8')
	plt.plot(time[:step], output_fuel[:,8], color[8], label = 'car9')
	plt.title('Fuel consumption')
	plt.xlabel('Time (s)')
	plt.ylabel('Fuel consumption (mile/gallon)')

	plt.show()




















