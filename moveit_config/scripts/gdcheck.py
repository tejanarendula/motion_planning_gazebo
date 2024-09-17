import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import casadi
import DynamicsNew
from matplotlib.animation import FuncAnimation
import roboticstoolbox as rtb

def euclidean_distance(x1, y1, z1, x2, y2, z2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def nav_func(x, y, z, tx, ty, tz, ox, oy, oz,K,ri):
    phi = euclidean_distance(x,y,z,tx,ty,tz)**2/((euclidean_distance(x,y,z,tx,ty,tz)**(2*K) + euclidean_distance(x,y,z,ox,oy,oz)**2-ri**2)**(1/K))
    return phi

def partial_x_nav(x, y, z, tx, ty, tz, ox, oy, oz,K,ri):
    c1 = euclidean_distance(x,y,z,tx,ty,tz)**(2*K) + euclidean_distance(x,y,z,ox,oy,oz)**2 - ri**2
    c2= c1**(-1/K - 1)

    c3 = 2*K*euclidean_distance(x,y,z,tx,ty,tz)**(2*K - 2)*(x-tx) + 2*(x-ox)

    c4 = c1**(-1/K)

    return -euclidean_distance(x,y,z,tx,ty,tz)**2*c2*c3/K + c4*2*(x-tx)

def partial_y_nav(x, y, z, tx, ty, tz, ox, oy, oz,K,ri):
    c1 = euclidean_distance(x,y,z,tx,ty,tz)**(2*K) + euclidean_distance(x,y,z,ox,oy,oz)**2 - ri**2
    c2= c1**(-1/K - 1)

    c3 = 2*K*euclidean_distance(x,y,z,tx,ty,tz)**(2*K - 2)*(y-ty) + 2*(y-oy)

    c4 = c1**(-1/K)

    return -euclidean_distance(x,y,z,tx,ty,tz)**2*c2*c3/K + c4*2*(y-ty)

def partial_z_nav(x, y, z, tx, ty, tz, ox, oy, oz,K,ri): 
    c1 = euclidean_distance(x,y,z,tx,ty,tz)**(2*K) + euclidean_distance(x,y,z,ox,oy,oz)**2 - ri**2
    c2= c1**(-1/K - 1)

    c3 = 2*K*euclidean_distance(x,y,z,tx,ty,tz)**(2*K - 2)*(z-tz) + 2*(z-oz)

    c4 = c1**(-1/K)

    return -euclidean_distance(x,y,z,tx,ty,tz)**2*c2*c3/K + c4*2*(z-tz)

    
# Defining main function and partial derivatives
def f(x, y, z,tx,ty,tz,ox,oy,oz):
    return 5*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)-4*np.exp(-((x-tx)**2+(y-ty)**2 + (z-tz)**2)/0.1) + ((x-tx)**2 + (y-tx)**2+(z-tx)**2)/5
    #return -4*np.exp(-((x-0.5)**2+(y-0.5)**2 + (z-0.5)**2)/0.1) + ((x-0.5)**2 + (y-0.5)**2+(z-0.5)**2)/5
def partial_x(x, y, z,tx,ty,tz,ox,oy,oz):
    return  5*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(x-ox)/0.1)-4*np.exp(-((x-tx)**2+(y-ty)**2+ (z-tz)**2)/0.1)*(-2*(x-tx)/0.1) + 2*(x-tx)/5

def partial_y(x, y, z,tx,ty,tz,ox,oy,oz):
    return 5*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(y-oy)/0.1)-4*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2*(y-ty)/0.1) + 2*(y-ty)/5

def partial_z(x, y, z,tx,ty,tz,ox,oy,oz):
    return 5*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(z-oz)/0.1)-4*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2*(z-tz)/0.1) + 2*(z-tz)/5

def partial_x_x(x,y,z,tx,ty,tz,ox,oy,oz):
    return 5*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2/0.1) + 5*(-2*(x-ox)/0.1)*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(x-ox)/0.1) - 4*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2/0.1) - 4*(-2*(x-tx)/0.1)*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2*(x-tx)/0.1) + 2/5

def partial_x_y(x,y,z,tx,ty,tz,ox,oy,oz):
    return 5*(-2*(y-oy)/0.1)*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(x-ox)/0.1) - 4*(-2*(y-ty)/0.1)*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2*(x-tx)/0.1)

def partial_x_z(x,y,z,tx,ty,tz,ox,oy,oz):
    return 5*(-2*(z-oz)/0.1)*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(x-ox)/0.1) - 4*(-2*(z-tz)/0.1)*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2*(x-tx)/0.1)

def partial_y_y(x,y,z,tx,ty,tz,ox,oy,oz):
    return 5*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2/0.1) + 5*(-2*(y-oy)/0.1)*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(y-oy)/0.1) - 4*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2/0.1) - 4*(-2*(y-ty)/0.1)*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2*(y-ty)/0.1) + 2/5

def partial_y_z(x,y,z,tx,ty,tz,ox,oy,oz):
    return 5*(-2*(z-oz)/0.1)*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(y-oy)/0.1) - 4*(-2*(z-tz)/0.1)*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2*(y-ty)/0.1)

def partial_z_z(x,y,z,tx,ty,tz,ox,oy,oz):
    return 5*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2/0.1) + 5*(-2*(z-oz)/0.1)*np.exp(-((x-ox)**2+(y-oy)**2+(z-oz)**2)/0.1)*(-2*(z-oz)/0.1) - 4*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2/0.1) - 4*(-2*(z-tz)/0.1)*np.exp(-((x-tx)**2+(y-ty)**2+(z-tz)**2)/0.1)*(-2*(z-tz)/0.1) + 2/5

def hessian(x,y,z,tx,ty,tz,ox,oy,oz):
    return ([partial_x_x(x,y,z,tx,ty,tz,ox,oy,oz), partial_x_y(x,y,z,tx,ty,tz,ox,oy,oz), partial_x_z(x,y,z,tx,ty,tz,ox,oy,oz)],
            [partial_x_y(x,y,z,tx,ty,tz,ox,oy,oz), partial_y_y(x,y,z,tx,ty,tz,ox,oy,oz), partial_y_z(x,y,z,tx,ty,tz,ox,oy,oz)],
            [partial_x_z(x,y,z,tx,ty,tz,ox,oy,oz), partial_y_z(x,y,z,tx,ty,tz,ox,oy,oz), partial_z_z(x,y,z,tx,ty,tz,ox,oy,oz)])


# Start coordinate
x = 0.8
y = 0.7
z = 0.8

ox = 0.2
oy = 0.2
oz = 0.2

tx = 0.1
ty = 0.1
tz = 0.1

K=8
ri=0.01

px =[]
py =[]
pz =[]
J = []

lr = 0.02  # Learning rate
max_iter = 1000  # Maximum number of iterations

# Run gradient descent
for _ in range(max_iter):
    # Calculate partial derivatives
    dx = partial_x_nav(x, y, z,tx,ty,tz,ox,oy,oz,K,ri)
    dy = partial_y_nav(x, y, z,tx,ty,tz,ox,oy,oz,K,ri)
    dz = partial_z_nav(x, y, z,tx,ty,tz,ox,oy,oz,K,ri)

    # # Check for minima
    # grad = [dx, dy, dz]
    # H = hessian(x,y,z,tx,ty,tz,ox,oy,oz)
    # eigv = np.linalg.eig(H)

    # # Check for minima
    # if grad[0] == 0 and grad[1] == 0 and grad[2] == 0:
    #         #Random perturbation
    #         # Random perturbation function

    #         perturbation = np.random.randn(3) * 0.01
    #         x += perturbation[0]
    #         y += perturbation[1]
    #         z += perturbation[2]
    # else:
    #     # Update current point using usual gradient vector
    x -= lr * dx
    y -= lr * dy
    z -= lr * dz

    # Store path
    px.append(x)
    py.append(y)
    pz.append(z)
    J.append(nav_func(x,y,z,tx,ty,tz,ox,oy,oz,K,ri))

    # Print current point
    print(f"Iteration {_ + 1}: x = {x}, y = {y}, z = {z} ")
    # print(f"Cost: {f(x,y,z)}")
    # print(f"Gradient: {dx, dy, dz}")
    # print(f"Hessian: {H}")
    

    # Check convergence criteria
    if abs(x-tx) < 0.001 and abs(y-ty) < 0.001 and abs(z-tz) < 0.001:
        print("Converged")
        break

print("Path generated")

#Plot the path
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(px, py, pz, marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


#Add sphere of radius ri at ox,oy,oz
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = ri * np.outer(np.cos(u), np.sin(v)) + ox
y = ri * np.outer(np.sin(u), np.sin(v)) + oy
z = ri * np.outer(np.ones(np.size(u)), np.cos(v)) + oz
ax.plot_surface(x, y, z, color='b')
plt.show()

#Plot the cost
plt.plot(J)
plt.xlabel('Iterations')
plt.ylabel('Cost')
plt.title('Cost vs. Iterations')
plt.show()

xlist = px
ylist = py
zlist = pz

#Weights
wtrack=10 #Looks good
wangle =1
R=np.zeros((6,6),dtype=float)
R[0][0] = 10^10
R[1][1] = 10^10
R[2][2] = 10^11
R[3][3] = 10^11
R[4][4] = 10^11
R[5][5] = 0

#Limits
qmin= -2*np.pi
qmax= 2*np.pi

#MPC parameters
Np=5
Nc=3
Ts=0.1
Ntotal= len(xlist)-1

#Main matrices
# u is the 5N x Np+1 matrix of control inputs --> to be optimized
# x0 is the 11N x 1 vector at time k =0
# cost is the total cost of the system till time k = Np

referencevector = np.zeros((3,1),dtype=float)

referencevector[0][0]=xlist[0]
referencevector[1][0]=ylist[0]
referencevector[2][0]=zlist[0]


xhistory = np.zeros((3,Ntotal+1),dtype=float) # Except Xref,Yref

uhistory = np.zeros((6,Ntotal+1),dtype=float)

qhistory = np.zeros((6,Ntotal+1),dtype=float)

#Write the initial conditions into the history
for i in range(0,3):
    xhistory[i][0]=referencevector[i][0]

opti=casadi.Opti()

R_casadi = casadi.DM(R)

for t in range(1,Ntotal+1):

    print("Iteration: ",t)

    u_casadi=opti.variable(6,Np) #From t=k to t=k+Np

    x_casadi=opti.variable(3,Np) #From t=k+1 to t=k+Np, except Xref,Yref..

    q_casadi=opti.variable(6,Np) #From t=k to t=k+Np

    #uex_casadi=opti.variable(6,Nc) #From t=k to t=k+Nc

    x0_casadi=opti.parameter(3,1) #At t=k

    # Warm start
    for i in range(0,3):
        opti.set_value(x0_casadi[i],xhistory[i][t-1])

    x_casadi[:,0]=DynamicsNew.FT(q_casadi[:,0])

    for i in range(1,Nc):
        q_casadi[:,i]=q_casadi[:,i-1]+Ts*u_casadi[:,i-1]
        x_casadi[:,i]=DynamicsNew.FT(q_casadi[:,i])

    #Explicit MPC
    for i in range(Nc,Np):
        u_casadi[:,i]=u_casadi[:,i-1]
    
    for i in range(Nc,Np):
        q_casadi[:,i]=q_casadi[:,i-1]+Ts*u_casadi[:,i-1]
        x_casadi[:,i]=DynamicsNew.FT(q_casadi[:,i])

    # Change the reference vector
    referencevector[0][0]=xlist[t]
    referencevector[1][0]=ylist[t]
    referencevector[2][0]=zlist[t]

    #Cost function
    cost = 0
    for i in range(0, Np):
        current_position = x_casadi[:,i]
        
        x_end_eff = current_position[0]
        y_end_eff = current_position[1]
        z_end_eff = current_position[2]

        #Input cost
        cost += u_casadi[:,i].T @ R_casadi @ u_casadi[:,i]

        #Joint angle cost
        cost += wangle*(q_casadi[:,i] - qhistory[:,t-1]).T @ (q_casadi[:,i] - qhistory[:,t-1])

        # Tracking cost
        cost += wtrack *((x_end_eff - referencevector[0])**2 +  (y_end_eff - referencevector[1])**2 + (z_end_eff - referencevector[2])**2)
    
    opti.minimize(cost)

    #Constraints
    for i in range(0,Np):
        for j in range(0,6):
            opti.subject_to(q_casadi[j,i] <= qmax)
            opti.subject_to(q_casadi[j,i] >= qmin)

    #Set initial value of all inputs as zero for fast computation
    for i in range(0,Np):
        opti.set_initial(u_casadi[:,i],uhistory[:,t-1]+1)
        #opti.set_initial(u_casadi[:,i],0)
         
    #Set initial value of all states as the previous value
        
    solver_opts = {"ipopt": {"print_level": 0, "linear_solver": "mumps"}}
    opti.solver('ipopt', solver_opts)

    print("Solving...")
    sol = opti.solve()

    print("Solved")
    #Store the first input from the solution
    uhistory[:,t]=sol.value(u_casadi[:,0])
    xhistory[:,t]=sol.value(x_casadi[:,0])
    qhistory[:,t]=sol.value(q_casadi[:,0])


    print('Position of end effector: ',xhistory[:,t])

    print('Target position: ',xlist[t],ylist[t],zlist[t])

    print('Joint angles: ',qhistory[:,t]) 

    print('Control input: ',uhistory[:,t])

    print("iteration: ",t," completed")



# Plots

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xlist, ylist, zlist, marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.plot(xhistory[0,:],xhistory[1,:],xhistory[2,:], marker='*')
plt.show()

for i in range(0,Ntotal+1):
    
    if abs(xhistory[0][i]-xlist[i]) > 0.1 or abs(xhistory[1][i]-ylist[i]) > 0.1 or abs(xhistory[2][i]-zlist[i]) > 0.1:
        print("Failed")
        print(xhistory[0][i],xlist[i],xhistory[1][i],ylist[i],xhistory[2][i],zlist[i])
        break

#Print qhistory as csv as 6 joint angles in Ntotal+1 rows
np.savetxt("qhistory.csv", qhistory, delimiter=",")

# Run simulation on robotics toolbox

# robot = rtb.models.UR5()

# joint_angles_sequence = qhistory.T
# # Animation parameters
# fps = 30  # Frames per second
# duration = 5  # Duration of animation in seconds

# # Function to update the plot for each frame
# def update(frame):
#     robot.q = joint_angles_sequence[frame]
#     return robot.plot(frame)

# # Create a figure and axis for the animation
# fig, ax = plt.subplots()
# ax.set_aspect('equal')
# ax.set_xlim([-2, 2])
# ax.set_ylim([-2, 2])

# # Create the animation
# ani = FuncAnimation(fig, update, frames=len(joint_angles_sequence), blit=False, repeat=False)

# # Save the animation as a video
# ani.save('robotic_arm.mp4', writer='ffmpeg', fps=fps)

# plt.show()

    