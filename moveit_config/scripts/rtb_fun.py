import roboticstoolbox as rtb
import numpy as np
import spatialmath.base as base


robot = rtb.models.DH.UR5()
print(robot)

q = [0, -np.pi/2, np.pi/2, 0, 0, 0]

print(robot.fkine(q))

print(robot.A(4,q).t)

qt = rtb.jtraj(robot.qz, q, 50)

robot.plot(qt.q, backend ='pyplot', block=False)