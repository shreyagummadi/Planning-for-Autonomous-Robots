import random
import math
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt
from IPython import get_ipython
get_ipython().run_line_magic('matplotlib','qt')
import time

''' can tune these parameters to get results'''
sample = 1000  # number of sample points
Knn = 15  # number of neighbours
length= 50.0  # Maximum edge length

show_animation = True


class Node:
    def __init__(self, x, y, cost, p_ind):  # store node x,y co-ordinate, cost and parent index
        self.x = x
        self.y = y
        self.cost = cost
        self.p_ind = p_ind


class KDTree:
    # search for K nearest neighbours
    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):

        if len(inp.shape) >= 2:  
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r): # point within r distance taken into consideration

        index = self.tree.query_ball_point(inp, r)
        return index
    
def generate_sample_points(ix, iy, gx, gy, r, obs_x, obs_y, obkdtree):
    max_x = max(obs_x)
    max_y = max(obs_y)
    min_x = min(obs_x)
    min_y = min(obs_y)

    x_sample, y_sample = [], []

    while len(x_sample) <= sample:
        tx = (random.random() - min_x) * (max_x - min_x)
        ty = (random.random() - min_y) * (max_y - min_y)

        index, dist = obkdtree.search(np.array([tx, ty]).reshape(2, 1))

        if dist[0] >= r:
            x_sample.append(tx)
            y_sample.append(ty)

    x_sample.append(ix)
    y_sample.append(iy)
    x_sample.append(gx)
    y_sample.append(gy)

    return x_sample, y_sample 
   
def collides(ix, iy, gx, gy, r, okdtree):
    x = ix
    y = iy
    dx = gx - ix
    dy = gy - iy
    theta = math.atan2(gy - iy, gx - ix)
    d = np.sqrt(dx**2 + dy**2)

    if d >= length:
        return True

    D = r
    n = int(round(d / D))
#    print(n)
    for i in range(n):
        index, dist = okdtree.search(np.array([x, y]).reshape(2, 1))
        if dist[0] <= r:
            return True  
        x += D * np.cos(theta)
        y += D * np.sin(theta)

    index, dist = okdtree.search(np.array([gx, gy]).reshape(2, 1))
    if dist[0] <= r:
        return True 

    return False 


def generate_roadmap(x_sample, y_sample, r, obstacle_kdtree):
    road_map = []
    nsample = len(x_sample)
    skdtree = KDTree(np.vstack((x_sample, y_sample)).T)

    for (i, ix, iy) in zip(range(nsample), x_sample, y_sample):

        index, dists = skdtree.search(
            np.array([ix, iy]).reshape(2, 1), k=nsample)
        ind = index[0]
        edge_id = []

        for ii in range(1, len(ind)):
            nx = x_sample[ind[ii]]
            ny = y_sample[ind[ii]]

            if not collides(ix, iy, nx, ny, r, obstacle_kdtree):
                edge_id.append(ind[ii])   #add the edge to the list

            if len(edge_id) >= Knn:      # find k nearest neighbours and then break
                break

        road_map.append(edge_id)
#    print(len(road_map))
    return road_map
            
''' to find optimal path we use dijkstra'''
def dijkstra(ix, iy, gx, gy, obs_x, obs_y, r, road_map, x_sample, y_sample):
    init = Node(ix, iy, 0.0, -1)  # parent id for start and goal node is -1
    goal = Node(gx, gy, 0.0, -1)

    unvisited = dict()
    visited = dict()
    unvisited[len(road_map) - 2] = init

    while True:
        if not unvisited:
            print("Cannot find path")
            break

        c_id = min(unvisited, key=lambda x: unvisited[x].cost)
        curr_node = unvisited[c_id]

         #node exploration animation
        if show_animation and len(visited.keys()) % 2 == 0:
            plt.plot(curr_node.x, curr_node.y, "xg")
            plt.pause(0.01)

        if c_id == (len(road_map) - 1):
            print("Reached Goal!!")
            goal.p_ind = curr_node.p_ind
            goal.cost = curr_node.cost
            break

        # Remove the item from the open set
        del unvisited[c_id]
        # Add it to the closed set
        visited[c_id] = curr_node

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]    #generate node id
            dx = x_sample[n_id] - curr_node.x
            dy = y_sample[n_id] - curr_node.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(x_sample[n_id], y_sample[n_id],
                        curr_node.cost + d, c_id)

            if n_id in visited:
                continue
            # if it is already in the open set
            if n_id in unvisited:
                if unvisited[n_id].cost > node.cost:
                    unvisited[n_id].cost = node.cost  # update the cost
                    unvisited[n_id].p_ind = c_id       # update the parent id
            else:
                unvisited[n_id] = node
#    print(len(visited))
    # final path
    rx, ry = [goal.x], [goal.y]    # get the optimal path nodes
    p_ind = goal.p_ind
    while p_ind != -1:
        n = visited[p_ind]
        rx.append(n.x)
        ry.append(n.y)
        p_ind = n.p_ind
    return rx, ry


def PRM(ix, iy, gx, gy, obs_x, obs_y, r):

    obstacle_kdtree = KDTree(np.vstack((obs_x, obs_y)).T)

    x_sample, y_sample = generate_sample_points(ix, iy, gx, gy, r, obs_x, obs_y, obstacle_kdtree) # get sample points
    if show_animation:
        plt.plot(x_sample, y_sample, ".b")

    road_map = generate_roadmap(x_sample, y_sample, r,obstacle_kdtree) # generate roadmap from sample points by exploring closest neighbours

    rx, ry = dijkstra(ix, iy, gx, gy, obs_x, obs_y, r, road_map, x_sample, y_sample) # run dijkstra algorithm on the roadmap to get the optimal path

    return rx, ry



obs_x = []
obs_y = []
''' creating the outer wall of the maze'''
for i in range(251):
    obs_x.append(i)
    obs_y.append((0))
for i in range(151):
    obs_x.append((0))
    obs_y.append(i)
for i in range(251):
    obs_x.append(i)
    obs_y.append((150))
for i in range(151):
    obs_x.append((250))
    obs_y.append(i)
    
''' creating the maze''' 

for i in range(31):
    obs_x.append(10)
    obs_y.append(150-i)
for i in range(41):
    obs_x.append(i)
    obs_y.append(110)
for i in range(21):
    obs_x.append(40)
    obs_y.append(110+i)
for i in range(61):
    obs_x.append(20)
    obs_y.append(110-i)
for i in range(31):
    obs_x.append(20+i)
    obs_y.append(50)
for i in range(21):
    obs_x.append(50)
    obs_y.append(50-i)
for i in range(41):
    obs_x.append(50-i)
    obs_y.append(30)
for i in range(16):
    obs_x.append(30)
    obs_y.append(30-i)
for i in range(11):
    obs_x.append(30-i)
    obs_y.append(15)
for i in range(11):
    obs_x.append(30+i)
    obs_y.append(16)
for i in range(70):
    obs_x.append(56)
    obs_y.append(150-i)
for i in range(16):
    obs_x.append(55-i)
    obs_y.append(80)
for i in range(21):
    obs_x.append(100)
    obs_y.append(120-i)
for i in range(26):
    obs_x.append(100+i)
    obs_y.append(100)
for i in range(81):
    obs_x.append(125)
    obs_y.append(100-i)
for i in range(31):
    obs_x.append(125+i)
    obs_y.append(60)
for i in range(21):
    obs_x.append(155)
    obs_y.append(60+i)
for i in range(21):
    obs_x.append(155+i)
    obs_y.append(80)
for i in range(31):
    obs_x.append(125-i)
    obs_y.append(20)
for i in range(21):
    obs_x.append(95)
    obs_y.append(20-i)
for i in range(41):
    obs_x.append(150)
    obs_y.append(150-i)
for i in range(41):
    obs_x.append(150-i)
    obs_y.append(130)
for i in range(31):
    obs_x.append(175)
    obs_y.append(i)
for i in range(16):
    obs_x.append(175-i)
    obs_y.append(30)
for i in range(51):
    obs_x.append(200)
    obs_y.append(150-i)
for i in range(31):
    obs_x.append(175)
    obs_y.append(80+i)
for i in range(26):
    obs_x.append(250-i)
    obs_y.append(60)
for i in range(11):
    obs_x.append(225)
    obs_y.append(60+i)
for i in range(31):
    obs_x.append(225)
    obs_y.append(60-i)
for i in range(16):
    obs_x.append(225-i)
    obs_y.append(30)
for i in range(31):
    obs_x.append(250-i)
    obs_y.append(120)
for i in range(31):
    obs_x.append(80)
    obs_y.append(80-i)
for i in range(16):
    obs_x.append(80+i)
    obs_y.append(65)
    
''' generating random obstacles in maze'''
''' comment out this part to get only the maze path used for simulation'''
for i in range(20):   # generating 20 random circular obstacles in the maze
    r = np.random.randint(1,5)
    x = np.random.randint(10,200)
    y = np.random.randint(10,140)
    for theta in np.arange(0,2*np.pi,0.06):
        obs_x.append((x+r*np.cos(theta)))
        obs_y.append((y+r*np.sin(theta)))


# start and goal position
init_x = 10
init_y = 10
goal_x = 245
goal_y = 145
r =1.72

if show_animation:
    plt.plot(obs_x, obs_y, ".k")
    plt.plot(init_x, init_y, "*r")
    plt.plot(goal_x, goal_y, "*r")
    plt.grid(True)
    plt.axis("equal")
    


start = time.time()
rx, ry = PRM(init_x,init_y, goal_x, goal_y, obs_x, obs_y, r)
end = time.time()
print('Time taken to run:'+ str(end - start))
assert rx, 'Path cannot be found!!'
plt.plot(rx, ry, "-r")  #plot final trajectory
plt.show()

x = open('x.txt','w') # save the x and y co-ordinates to text file
y = open('y.txt','w')
for i in range(len(rx)):
    x.write(str(rx[i])+'\n')
    y.write(str(ry[i])+'\n')