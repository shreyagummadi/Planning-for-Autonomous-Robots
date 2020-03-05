import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mp
import time
from IPython import get_ipython
get_ipython().run_line_magic('matplotlib','qt')
show_animation = True

res = int(input('Enter resolution:')) #input as 1
ix,iy= input('Enter starting point:').split(',')#input as 5,3
ix,iy = int(ix),int(iy)
gx,gy = input('Enter goal point:').split(',') #input as 10,50
gx,gy = int(gx),int(gy)
r = int(input('Enter robot radius:')) #input as 2
c = int(input('Enter clearance value:')) #input as 3
class Node:
    def __init__(self,x,y,cost,p_ind):
        self.x = x  # x co-ordinate
        self.y = y  # y co-ordinate
        self.cost = cost  # cost
        self.p_ind = p_ind # parent index (-1 for start and goal)

#    
def movement():
    # x, y, cost
    motion = [[1, 0, 1],[0, 1, 1],[-1, 0, 1], [0, -1, 1],[-1, -1, np.sqrt(2)],[-1, 1, np.sqrt(2)],[1, -1, np.sqrt(2)],[1, 1, np.sqrt(2)]]

    return motion

def heuristic(node1,node2):
    '''using euclidean distance'''
    dist = np.sqrt((node1.x-node2.x)**2+(node1.y-node2.y)**2)
    return dist

'''check if the node is in the obstcale free space'''  
def exists(curr_node,res,c,r):
    x,y = curr_node.x,curr_node.y
    exist = True

    poly = [[(125-c-r)/res,(56+c+r)/res],[163/res,(52+c+r)/res],[(170)/res,(90+c+r)/res],[(193+c+r)/res,52/res],[(173+c+r)/res,(15-c-r)/res],[(150-c-r)/res,(15-c-r)/res]]
    l1 = mp.Path(poly)
    l2 = (x-190/res)**2 +(y-130/res)**2-((15+c+r)/res)**2
    l3 = ((x-140/res)**2/((15+c+r)/res)**2)+((y-120/res)**2/((6+c+r)/res)**2)-1
    if (x>=(0+c+r)/res and x<=(250-c-r)/res and y>=(0+c+r)/res and y<=(150-c-r)/res):
        if (x>=(50-c-r)/res and x<=(100+c+r)/res and y>=(67.5-c-r)/res and y<=(112.5+c+r)/res):
            exist = False
        elif l1.contains_point((x,y))==True:
                exist = False
        elif l2<=0:
            exist = False
        elif l3<= 0:
            exist = False
    else:
        exist = False
    return exist

''' get a unique id for each node'''
def generate_id(curr_node):
    return (curr_node.y)*250 + (curr_node.x)

def plot():
    yaxis = (150) /res
    xaxis = (250)/res
    plt.axis([0,xaxis,0,yaxis])
    plt.xticks(np.arange(0,xaxis,20)) # define the step count for x axis
    plt.yticks(np.arange(0,yaxis,20)) # define the step count for y axis
    #        plt.axis('equal')
    plt.grid(True)
    
    ''' Square obstacle'''
    x1 = np.linspace((50-c-r)/res,(100+c+r)/res,40)
    y1 = ((67.5-c-r)/res)*np.ones(len(x1))
    y2 = ((112.5+c+r)/res)*np.ones(len(x1))
    
    y3 = np.linspace((67.5-c-r)/res,(112.5+c+r)/res,40)
    x3 = ((50-c-r)/res)*np.ones(len(y3))
    x4 = ((100+c+r)/res)*np.ones(len(y3))
    
    plt.fill([x1,x3,x1,x4],[y1,y3,y2,y3],'b')
    
    '''Polygon obstacle'''
    x1 = np.linspace((125-c-r)/res,163/res,30)
    y1 = ((((56+c+r)-(52+c+r))/((125-c-r)-(163)))*(x1-163/res)+(52+c+r)/res)
    
    x2 = np.linspace(163/res,170/res,30)
    y2 = ((((90+c+r)-(52+c+r))/(170-163))*(x2-170/res)+(90+c+r)/res)
    
    x3 = np.linspace(170/res,(193+c+r)/res,30)
    y3 = ((((90+c+r)-(52))/(170-(193+c+r)))*(x3-170/res)+(90+c+r)/res)
    
    x4 = np.linspace((193+c+r)/res,(173+c+r)/res,30)
    y4 = (((52-(15-c-r))/((193+c+r)-(173+c+r)))*(x4-(193+c+r)/res)+52/res)
    
    x5 = np.linspace((173+c+r)/res,(150-c-r)/res,30)
    y5 = (((15-c-r)/res)*np.ones(len(x5)))
    
    x6 = np.linspace((125-c-r)/res,(150-c-r)/res,30)
    y6 = ((((56+c+r)-(15-c-r))/((125-c-r)-(150-c-r)))*(x6-(125-c-r)/res)+(56+c+r)/res)
    
    x7 = np.linspace(163/res,(173+c+r)/res,30)
    y7 = (((15-c-r)-(52+c+r))/((173+c+r)-163))*(x7-(173+c+r)/res)+(15-c-r)/res
    plt.fill([x1,x7,x5,x6],[y1,y7,y5,y6],'b')
    plt.fill([x7,x2,x3,x4],[y7,y2,y3,y4],'b')
    
    ''' Ellipse obstacle'''
    theta = np.linspace(0,2*np.pi,100)
    xellipse = ((15+c+r)*np.cos(theta)+140)/res
    yellipse = ((6+c+r)*np.sin(theta)+120)/res
    plt.fill(xellipse,yellipse,'b')
    
    ''' Circle obstacle'''
    xcircle = (190+(15+c+r)*np.cos(theta))/res
    ycircle = (130+(15+c+r)*np.sin(theta))/res
    plt.fill(xcircle,ycircle,'b')


def astar(ix,iy,gx,gy,res,c,r):
    init = Node((ix/res),(iy/res),0,-1)
    goal = Node((gx/res),(gy/res),0,-1)
    motion = movement()
    unvisited = dict()
    visited = dict()
    unvisited[generate_id(init)]=init
    while 1:
        if exists(init,res,c,r)==False:
            print('Initial position not possible')
            break
        if exists(goal,res,c,r)==False:
            print('Goal position not possible')
            break
        c_id =  min(unvisited,key=lambda x:unvisited[x].cost+heuristic(goal,unvisited[x]))
        curr_node = unvisited[c_id]
#        if show_animation:     '''Uncomment to get node exploration animation'''
#            plot()
#            plt.title('Animation')
#            plt.plot(ix/res,iy/res,'r*')
#            plt.plot(gx/res,gy/res,'r*')
#            plt.plot(curr_node.x , curr_node.y, "xc")
#            if len(visited.keys()) % 10 == 0:
#                plt.pause(0.001)
        if curr_node.x ==goal.x and curr_node.y==goal.y:
            print('Goal found')
            goal.p_ind = curr_node.p_ind
            goal.cost = curr_node.cost
            break
        del unvisited[c_id]
        visited[c_id] = curr_node
        for i, _ in enumerate(motion):
            node = Node(curr_node.x + motion[i][0], curr_node.y + motion[i][1],
                        curr_node.cost + motion[i][2], c_id)
            n_id = generate_id(node) # get the node id
            if exists(node,res,c,r)==False:   # if the node is not in the free space exit the loop
                continue
            if n_id in visited:
                continue
            if n_id not in unvisited:
                unvisited[n_id] = node
            else:
                if unvisited[n_id].cost >= node.cost:  #update the cost
                    unvisited[n_id] = node
    

    rx, ry = [goal.x], [goal.y] # get the optimal path nodes
    p_ind = goal.p_ind
    while p_ind != -1:
        n = visited[p_ind]
        rx.append(n.x)
        ry.append(n.y)
        p_ind = n.p_ind
    return (rx,ry)

start = time.time()
rx,ry = (astar(ix,iy,gx,gy,res,c,r))
end = time.time()
print('Time taken to run:'+ str(end - start))
plot()
plt.plot(ix/res,iy/res,'r*')
plt.plot(gx/res,gy/res,'r*')
plt.plot(rx,ry,'r-')
plt.title('Animation')
plt.show()