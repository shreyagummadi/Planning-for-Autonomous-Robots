import numpy as np
import time
import matplotlib.pyplot as plt
from IPython import get_ipython
get_ipython().run_line_magic('matplotlib','qt')



''' all dimensions in m'''
show_animation = True

ix,iy= input('Enter starting point:').split(',')#input as 5,3
ix,iy = float(ix),float(iy)
gx,gy = input('Enter goal point:').split(',') #input as 10,50
gx,gy = float(gx),float(gy)

class Node:
    def __init__(self,x,y,theta,ul,ur,cost,p_ind):
        self.x = x  # x co-ordinate
        self.y = y  # y co-ordinate
        self.theta = theta #angle
        self.ul = ul #left wheel 
        self.ur = ur #right wheel
        self.cost = cost  # cost
        self.p_ind = p_ind # parent index (-1 for start and goal)

def generate_id(curr_node):
    return (curr_node.y)*250 + (curr_node.x)

def heuristic(node1,node2):
    '''using euclidean distance'''
    dist = np.sqrt((node1.x-node2.x)**2+(node1.y-node2.y)**2)
    return dist

def diff_constraint(ul,ur,curr_node,t=15):
    r = 0.038  # wheel radius
    L = 0.23   # distance between wheels
    x1,y1,theta1 = curr_node.x,curr_node.y,curr_node.theta #previous values of x,y and theta
    
    ur = (ur*2*np.pi)/60   # converting rpm to m/s
    ul = (ul*2*np.pi)/60   
    
    theta = (r/L)*(ur-ul)*t+theta1
    thetadot = ((r/L)*(ur-ul))
    
    x = (r/2)*(ur+ul)*np.cos(theta)*t +x1
    xdot = ((r/2)*(ur+ul)*np.cos(theta))
    
    y =(r/2)*(ur+ul)*np.sin(theta)*t +y1
    ydot = ((r/2)*(ur+ul)*np.sin(theta))
    
#    xy = np.hstack((x,y,theta))
    vel = np.hstack((xdot,ydot,thetadot))
    
    return x,y,theta, vel #new x,y,theta and velocity


def moves(u1,u2):
    
    u1 = (u1*2*np.pi)/60
    u2 = (u2*2*np.pi)/60
    motion = [[0,u1,0.0458],[u1,0,0.0458],[u1,u1,0.0916],[0,u2,0.0916],[u2,0,0.0916],[u2,u2,0.1833],[u1,u2,0.1374],[u2,u1,0.1374]]
    
    
    return motion
    
def obstacles():
    ''' inner wall 11.1 x 10.1 m'''
    plt.axis([-5.55,5.55,-5.05,5.05])
    
    ''' defining obstacles'''
    r = 0.177 # robot radius
    c = 0.05  # clearance
    x1 = [-4.0505-r-c,-2.4527+r+c,-2.4527+r+c,-4.0505-r-c]
    y1 = [4.05+r+c,4.05+r+c,2.451-r-c,2.451-r-c]
    plt.fill(x1,y1,'b')
    theta = np.linspace(0,2*np.pi,1000)
    xcircle1 = ((-4.0505)+(0.7995+r+c)*np.cos(theta))
    ycircle1 = ((3.2505)+(0.7995+r+c)*np.sin(theta))
    plt.fill(xcircle1,ycircle1,'b')
    
    xcircle2 = (-2.4527+(0.7995+r+c)*np.cos(theta))
    ycircle2 = (3.2505+(0.7995+r+c)*np.sin(theta))
    plt.fill(xcircle2,ycircle2,'b')
    
    xcircle3 = (-1.65+(0.405+r+c)*np.cos(theta))
    ycircle3 = (4.6+(0.405+r+c)*np.sin(theta))
    plt.fill(xcircle3,ycircle3,'b')
    
    xcircle4 = (-1.17+(0.405+r+c)*np.cos(theta))
    ycircle4 = (2.31+(0.405+r+c)*np.sin(theta))
    plt.fill(xcircle4,ycircle4,'b')
    
    xcircle5 = (-1.17+(0.405+r+c)*np.cos(theta))
    ycircle5 = (-2.31+(0.405+r+c)*np.sin(theta))
    plt.fill(xcircle5,ycircle5,'b')
    
    xcircle6 = (-1.65+(0.405+r+c)*np.cos(theta))
    ycircle6 = (-4.6+(0.405+r+c)*np.sin(theta))
    plt.fill(xcircle6,ycircle6,'b')
    
    x2 = [-1.17-r-c,-0.26+r+c,-0.26+r+c,-1.17-r-c]
    y2 = [-0.07+r+c,-0.07+r+c,-1.9-r-c,-1.9-r-c]
    plt.fill(x2,y2,'b')
    
    x3 = [-0.255-r-c,1.575+r+c,1.575+r+c,-0.255-r-c]
    y3 = [-1.64+r+c,-1.64+r+c,-2.4-r-c,-2.4-r-c]
    plt.fill(x3,y3,'b')
    
    x4 = [2.3-r-c,3.82+r+c,3.82+r+c,2.3-r-c]
    y4 = [-1.21+r+c,-1.21+r+c,-2.38-r-c,-2.38-r-c]
    plt.fill(x4,y4,'b')
    
    x5 = [2.77-r-c,3.63+r+c,3.63+r+c,2.77-r-c]
    y5 = [5.05+r+c,5.05+r+c,3.22-r-c,3.22-r-c]
    plt.fill(x5,y5,'b')
    
    x6 = [4.28-r-c,4.71+r+c,4.71+r+c,4.28-r-c]
    y6 = [5.05+r+c,5.05+r+c,4.14-r-c,4.14-r-c]
    plt.fill(x6,y6,'b')
    
    x7 = [1.89-r-c,5.55+r+c,5.55+r+c,1.89-r-c]
    y7 = [1.92+r+c,1.92+r+c,1.16-r-c,1.16-r-c]
    plt.fill(x7,y7,'b')
    
    x8 = [4.97-r-c,5.55+r+c,5.55+r+c,4.97-r-c]
    y8 = [0.31+r+c,0.31+r+c,-0.27-r-c,-0.27-r-c]
    plt.fill(x8,y8,'b')
    
    x9 = [4.64-r-c,5.55+r+c,5.55+r+c,4.64-r-c]
    y9 = [-0.565+r+c,-0.565+r+c,-1.425-r-c,-1.425-r-c]
    plt.fill(x9,y9,'b')
    
    x10 = [4.97-r-c,5.55+r+c,5.55+r+c,4.97-r-c]
    y10 = [-2.0975+r+c,-2.0975+r+c,-3.2675-r-c,-3.2675-r-c]
    plt.fill(x10,y10,'b')
    
    x11 = [3.72-r-c,5.55+r+c,5.55+r+c,3.72-r-c]
    y11 = [-3.94+r+c,-3.94+r+c,-4.7-r-c,-4.7-r-c]
    plt.fill(x11,y11,'b')
    
    x12 = [1.3-r-c,5.55+r+c,5.55+r+c,1.3-r-c]
    y12 = [-4.7+r+c,-4.7+r+c,-5.05-r-c,-5.05-r-c]
    plt.fill(x12,y12,'b')
    
    x13 = [2.24-r-c,3.41+r+c,3.41+r+c,2.24-r-c]
    y13 = [-4.12+r+c,-4.12+r+c,-4.7-r-c,-4.7-r-c]
    plt.fill(x13,y13,'b')
    
    x14 = [-0.81-r-c,1.93+r+c,1.93+r+c,-0.81-r-c]
    y14 = [-3.18+r+c,-3.18+r+c,-4.7-r-c,-4.7-r-c]
    plt.fill(x14,y14,'b')


def exists(curr_node):
    r = 0.177  #robot radius
    c = 0.05   # clearance
    x,y = curr_node.x, curr_node.y
    exist = True
    circle1 = (x-(-4.0505))**2+(y-3.2505)**2-(0.7995+r+c)**2
    circle2 = (x-(-2.4527))**2 +(y-3.2505)**2-(0.7995+r+c)**2
    circle3 = (x-(-1.65))**2+(y-4.6)**2-(0.405+r+c)**2
    circle4 = (x-(-1.17))**2+(y-2.31)**2-(0.405+r+c)**2
    circle5 = (x-(-1.17))**2+(y-(-2.31))**2-(0.405+r+c)**2
    circle6 = (x-(-1.65))**2+(y-(-4.6))**2-(0.405+r+c)**2
    if (x>=-5.55+r+c and x<=5.55-r-c and y>=-5.05+r+c and y<=5.05-r-c):
        if circle1<=0:
            exist = False
        elif circle2<=0:
            exist = False
        elif circle3<=0:
            exist = False
        elif circle4<=0:
            exist = False
        elif circle5<=0:
            exist = False
        elif circle6<=0:
            exist = False
        elif x>=-4.0505-r-c and x<=-2.4527+r+c and y<=4.05+r+c and y>=2.451-r-c:
            exist = False
        elif x>=-1.17-r-c and x<=-0.26+r+c and y<=-0.07+r+c and y>=-1.9-r-c:
            exist = False
        elif x>=-0.255-r-c and x<=1.575+r+c and y<=-1.64+r+c and y>=-2.4-r-c:
            exist = False
        elif x>=2.3-r-c and x<=3.82+r+c and y<=-1.21+r+c and y>=-1.9-r-c:
            exist = False
        elif x>=2.77-r-c and x<=3.63+r+c and y<=5.05+r+c and y>=3.22-r-c:
            exist = False
        elif x>=4.28-r-c and x<=4.71+r+c and y<=5.05+r+c and y>=4.14-r-c:
            exist = False
        elif x>=1.89-r-c and x<=5.55+r+c and y<=1.92+r+c and y>=1.16-r-c:
            exist = False
        elif x>=4.97-r-c and x<=5.55+r+c and y<=0.31+r+c and y>=-0.27-r-c:
            exist = False
        elif x>=4.64-r-c and x<=5.55+r+c and y<=-0.565+r+c and y>=-1.425-r-c:
            exist = False
        elif x>=4.97-r-c and x<=5.55+r+c and y<=-2.0975+r+c and y>=-3.2675-r-c:
            exist = False
        elif x>=3.72-r-c and x<=5.55+r+c and y<=-3.94+r+c and y>=-4.7-r-c:
            exist = False
        elif x>=1.3-r-c and x<=5.55+r+c and y<=-4.7+r+c and y>=-5.05-r-c:
            exist = False
        elif x>=2.24-r-c and x<=3.41+r+c and y<=-4.12+r+c and y>=-4.7-r-c:
            exist = False
        elif x>=-0.81-r-c and x<=1.93+r+c and y<=-3.18+r+c and y>=-4.7-r-c:
            exist = False
    else:
        exist = False
        
    return exist


def astar(ix,iy,gx,gy):
    u = (50*2*np.pi)/60
    v = (100*2*np.pi)/60
    init = Node((ix),(iy),0,u,v,0,-1)
    goal = Node((gx),(gy),0,0,0,0,-1)
    unvisited = dict()
    visited = dict()
    unvisited[generate_id(init)]=init
    u1 = 50
    u2 = 100
    motion = moves(u1,u2)
    while 1:
        if exists(init)==False:
            print('Initial position not possible')
            break
        if exists(goal)==False:
            print('Goal position not possible')
            break
        c_id =  min(unvisited,key=lambda x:unvisited[x].cost+heuristic(goal,unvisited[x]))
        curr_node = unvisited[c_id]
        if show_animation:     
            obstacles()
            plt.title('Animation')
            plt.plot(ix,iy,'r*')
            plt.plot(gx,gy,'r*')
            plt.plot(curr_node.x , curr_node.y, "xc")
            if len(visited.keys()) % 10 == 0:
                plt.pause(0.001)
        if (goal.x-curr_node.x)**2+(goal.y-curr_node.y)**2-(0.5)**2<=0:
            print('Goal found')
            goal.p_ind = curr_node.p_ind
            goal.cost = curr_node.cost
            break
        del unvisited[c_id]
        visited[c_id] = curr_node
        for i in range(len(motion)):
            x,y,theta,vel = diff_constraint(motion[i][0],motion[i][1],curr_node)
            node = Node(x,y,theta,motion[i][0],motion[i][1],curr_node.cost+motion[i][2],c_id)
            n_id = generate_id(node) # get the node id
            if exists(node)==False:   # if the node is not in the free space exit the loop
                continue
            if n_id in visited:
                continue
            if n_id not in unvisited:
                unvisited[n_id] = node
            else:
                if unvisited[n_id].cost >= node.cost:  #update the cost
                    unvisited[n_id] = node
            
    

    rx, ry,r_theta,left,right = [goal.x], [goal.y],[goal.theta],[goal.ul],[goal.ur] # get the optimal path nodes
    p_ind = goal.p_ind
    while p_ind != -1:
        n = visited[p_ind]
        rx.append(n.x)
        ry.append(n.y)
        r_theta.append(n.theta)
        left.append(n.ul)
        right.append(n.ur)
        p_ind = n.p_ind
    return rx,ry,left,right
      

start = time.time()
rx,ry,left,right= astar(ix,iy,gx,gy)
end = time.time()
print('Time taken to run:'+ str(end - start))

leftwheel = open('ul.txt','w') # save the velocities to text files
rightwheel = open('ur.txt','w')
for i in range(len(left)):
    leftwheel.write(str(left[i])+'\n')
    rightwheel.write(str(right[i])+'\n')

obstacles()
plt.plot(ix,iy,'r*')
plt.plot(gx,gy,'r*')
plt.plot(rx,ry,'r-')
plt.title('Animation')
plt.show()
#        