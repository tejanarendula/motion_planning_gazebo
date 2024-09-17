import numpy as np
import casadi
import roboticstoolbox as rtb
import spatialmath.base as base
import scipy as sp
import sympy as sym

def robot_FT(robot_model,N):

    qsympy = base.sym.symbol("q_:{}".format(N))
    
    T = robot_model.fkine(qsympy)

    #sympy expression

    x_coord = T.t[0]
    y_coord = T.t[1]
    z_coord = T.t[2]

    #Converting sympy expression to numpy expression
    x_coord_np = sym.lambdify(qsympy,x_coord,"numpy")
    y_coord_np = sym.lambdify(qsympy,y_coord,"numpy")
    z_coord_np = sym.lambdify(qsympy,z_coord,"numpy") 

    return x_coord_np, y_coord_np, z_coord_np

def FT(Xinput):

    q_0=Xinput[0]
    q_1=Xinput[1]
    q_2=Xinput[2]
    q_3=Xinput[3]
    q_4=Xinput[4]
    q_5=Xinput[5]
 
    #Casadi variable
    xyzfinal = casadi.MX.zeros(3,1)

    #Writing scipy function as as a casadi function
    xyzfinal[0] =  0.0823*np.sin(q_0)*np.cos(q_4) + 0.10915*np.sin(q_0) - 0.0823*np.sin(q_4)*np.cos(q_0)*np.cos(q_1 + q_2 + q_3)  + 0.09465*np.sin(q_1 + q_2 + q_3)*np.cos(q_0) - 0.425*np.cos(q_0)*np.cos(q_1) - 0.39225*np.cos(q_0)*np.cos(q_1 + q_2)
    xyzfinal[1] = -0.0823*np.sin(q_0)*np.sin(q_4)*np.cos(q_1 + q_2 + q_3) + 0.09465*np.sin(q_0)*np.sin(q_1 + q_2 + q_3) - 0.425*np.sin(q_0)*np.cos(q_1) - 0.39225*np.sin(q_0)*np.cos(q_1 + q_2) - 0.0823*np.cos(q_0)*np.cos(q_4) - 0.10915*np.cos(q_0)
    xyzfinal[2] = -0.425*np.sin(q_1) - 0.0823*np.sin(q_4)*np.sin(q_1 + q_2 + q_3) - 0.39225*np.sin(q_1 + q_2)- 0.09465*np.cos(q_1 + q_2 + q_3) + 0.089459

    
    return xyzfinal

##Usage

def dmin(qinput,po):

    robot= rtb.models.DH.UR5()

    p1 = robot.A(0,qinput).t
    p2 = robot.A(1,qinput).t
    p3 = robot.A(2,qinput).t
    p4 = robot.A(3,qinput).t
    p5 = robot.A(4,qinput).t
    p6 = robot.A(5,qinput).t

    d1 = np.sqrt((np.linalg.norm(p1)*np.linalg.norm(p1-po))**2-(np.dot(p1,p1-po))**2)/(np.linalg.norm(p1))
    d2 = np.sqrt((np.linalg.norm(p2-p1)*np.linalg.norm(p2-po))**2-(np.dot(p2-p1,p2-po))**2)/(np.linalg.norm(p2-p1))
    d3 = np.sqrt((np.linalg.norm(p3-p2)*np.linalg.norm(p3-po))**2-(np.dot(p3-p2,p3-po))**2)/(np.linalg.norm(p3-p2))
    d4 = np.sqrt((np.linalg.norm(p4-p3)*np.linalg.norm(p4-po))**2-(np.dot(p4-p3,p4-po))**2)/(np.linalg.norm(p4-p3))
    d5 = np.sqrt((np.linalg.norm(p5-p4)*np.linalg.norm(p5-po))**2-(np.dot(p5-p4,p5-po))**2)/(np.linalg.norm(p5-p4))
    d6 = np.sqrt((np.linalg.norm(p6-p5)*np.linalg.norm(p6-po))**2-(np.dot(p6-p5,p6-po))**2)/(np.linalg.norm(p6-p5))
    
    return d1,d2,d3,d4,d5,d6


#Read the scipy expression from FK.py, copy and paste it in the FT function

q = [ 1.04811588, -2.13787043, -1.0693871 , -0.28377357 , 0.42545352, -0.640697]
# print(FT(q)) 
po = [0.5,0.5,0.5]
    
robot= rtb.models.DH.UR5()

print(dmin(q,po))
