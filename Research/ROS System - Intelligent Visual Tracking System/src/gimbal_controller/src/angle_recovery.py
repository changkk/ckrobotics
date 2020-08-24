from scipy.optimize import fsolve
import math
import numpy as np

q0 = 0.0730045068521
q1 = 0.380819366482
q2 = -0.905278403605
q3 = -0.173545279563

targ_r = 18.9131
targ_p = 0
targ_y = -321.549

n_r = targ_r/0.02197265625/math.pi
n_p = targ_p/0.02197265625/math.pi
n_y = targ_y/0.02197265625/math.pi

while np.abs(n_r) >= 2:
    n_r = n_r-np.sign(n_r)*2

while np.abs(n_p) >= 2:
    n_p = n_p-np.sign(n_p)*2

while np.abs(n_y) >= 2:
    n_y = n_y-np.sign(n_y)*2

n_r = n_r*math.pi
n_p = n_p*math.pi
n_y = n_y*math.pi

def equations(v):
	r, p, y = v
	return (
    math.cos(y*0.5)*math.cos(r*0.5)*math.cos(p*0.5)+math.sin(y*0.5)*math.sin(r*0.5)*math.sin(p*0.5)-q0, 
    math.cos(y*0.5)*math.sin(r*0.5)*math.cos(p*0.5)-math.sin(y*0.5)*math.cos(r*0.5)*math.sin(p*0.5)-q1,
    math.cos(y*0.5)*math.cos(r*0.5)*math.sin(p*0.5)+math.sin(y*0.5)*math.sin(r*0.5)*math.cos(p*0.5)-q2)
    # math.sin(y*0.5)*math.cos(r*0.5)*math.cos(p*0.5)-math.cos(y*0.5)*math.sin(r*0.5)*math.sin(p*0.5)-q3)

r, p, y =  fsolve(equations, (0, 1, 1))

print equations((r, p, y))
print r/2
print p/2
print y/2
print "-----------------"
print n_r
print n_p
print n_y
print "-----------------"
print r*180/math.pi*0.02197265625
print p*180/math.pi*0.02197265625
print y*180/math.pi
print "-----------------"
print r*180/math.pi*0.02197265625
print p*180/math.pi*0.02197265625
print y*180/math.pi
print "-----------------"
print math.cos(y*0.5)*math.cos(r*0.5)*math.sin(p*0.5)+math.sin(y*0.5)*math.sin(r*0.5)*math.cos(p*0.5)-q2
print math.sin(y*0.5)*math.cos(r*0.5)*math.cos(p*0.5)-math.cos(y*0.5)*math.sin(r*0.5)*math.sin(p*0.5)-q3







	# return (
    # math.cos(y*0.5)*math.cos(r*0.5*0.02197265625)*math.cos(p*0.5*0.02197265625)+math.sin(y*0.5)*math.sin(r*0.5*0.02197265625)*math.sin(p*0.5*0.02197265625)-q0, 
    # math.cos(y*0.5)*math.sin(r*0.5*0.02197265625)*math.cos(p*0.5*0.02197265625)-math.sin(y*0.5)*math.cos(r*0.5*0.02197265625)*math.sin(p*0.5*0.02197265625)-q1,
    # # math.cos(y*0.5)*math.cos(r*0.5*0.02197265625)*math.sin(p*0.5*0.02197265625)+math.sin(y*0.5)*math.sin(r*0.5*0.02197265625)*math.cos(p*0.5*0.02197265625)-q2)
    # math.sin(y*0.5)*math.cos(r*0.5*0.02197265625)*math.cos(p*0.5*0.02197265625)-math.cos(y*0.5)*math.sin(r*0.5*0.02197265625)*math.sin(p*0.5*0.02197265625)-q3)
