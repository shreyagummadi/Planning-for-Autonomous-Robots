# -*- coding: utf-8 -*-
"""
Created on Tue May 14 19:10:39 2019

@author: Sneha
"""

# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 11:57:42 2019

@author: Sneha
"""

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import time
from IPython import get_ipython
get_ipython().run_line_magic('matplotlib','qt')
show_animation =False


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 expandDis=0.1, goalSampleRate=5, maxIter=30000):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.randx = randArea[0]
        self.randy = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self,n,circles,rectangles, animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.change_course(rnd, nind)
            #  print(newNode.cost)
#            if (np.abs(newNode.x - self.end.x)< (0.1) and np.abs(newNode.y - self.end.y)< (0.1)):
#               break;
            if self.verify_node(n,newNode, self.obstacleList):
#                print('true')
                nearinds = self.find_near_nodes(newNode)
                newNode = self.select_parent(n,newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(n,newNode, nearinds)
#            else:
#                print('False')
            if animation and i % 1000 == 0:
                self.DrawGraph(n,circles,rectangles,rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final(lastIndex)
        print(len(self.nodeList))
        return path

    def select_parent(self,n, newNode, nearinds):
        if not nearinds:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collide(n,self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def change_course(self, rnd, nind):

        # expand tree
        nearestNode = self.nodeList[nind]
     
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = Node(rnd[0], rnd[1])
        currentDistance = math.sqrt(
            (rnd[1] - nearestNode.y) ** 2 + (rnd[0] - nearestNode.x) ** 2)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= self.expandDis:
            pass
        else:
            newNode.x = nearestNode.x + self.expandDis * math.cos(theta)
            newNode.y = nearestNode.y + self.expandDis * math.sin(theta)
        newNode.cost = float("inf")
        newNode.parent = None
        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.randx[0], self.randx[1]),
                   random.uniform(self.randy[0], self.randy[1])]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]

        if not goalinds:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self,n, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 
                          2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collide(n,nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collide(self,n, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.verify_node(n,tmpNode, self.obstacleList):
                return False

        return True

    def DrawGraph(self,n,circles,rectangles, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")
#        plt.axis([0,0,250,150])
        plt.xticks(np.arange(-12.5,12.5,1))
        plt.yticks(np.arange(-7.5,7.5,1))
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
#        for (ox, oy, size) in self.obstacleList:
#            plt.plot(ox, oy, "ok", ms=30 * size)
#        fig, ax = plt.subplots()
        
        for i in range(5):
#            print(i[0])
#            print(i[3],i[4])
#            print(i[3])
            for j in range(n):
#                pri
                plt.fill(circles[3][j],circles[4][j], color = 'b' )
#                plt.fill(i[3],i[4], color = 'r' )
        for i in (rectangles):
#        
                for k in i[0]:
                    plt.fill(k[0],k[1], color = i[1])
                       
   
#         ax.legend()
#           ax.grid(color=(0,0,0), linestyle='-', linewidth=1) 
       
##        plt.axis([0,0,250,150])
#        plt.xticks(np.arange(0,25,2))
#        plt.yticks(np.arange(0,15,1))
#        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def verify_node(self,n, node, obstacleList):
#         global radius,clearance
        res=1
        radius=0.35/2
        clearance=0.1
        x=node.x
        y=node.y
    
        d=(radius)+(clearance)
        
        circ=[]
        for i in range(n):
#            print(obstacleList[1][0])
            circ.append(((x-(obstacleList[1][1][i]/res))*(x-(obstacleList[1][1][i]/res))+ (y-(obstacleList[1][2][i]/res))*(y-(obstacleList[1][2][i]/res)) - ((obstacleList[1][0][i]/res)+d)*((obstacleList[1][0][i]/res)+d)))
#        c2= ((x-(-1.17/res))*(x-(-1.17/res))+ (y-(2.31/res))*(y-(2.31/res)) - ((0.81/res)+d)*((0.81/res)+d))
#        c3= ((x-(-1.17/res))*(x-(-1.17/res))+ (y-(-2.31/res))*(y-(-2.31/res)) - ((0.81/res)+d)*((0.81/res)+d))
#        c4= ((x-(-1.65/res))*(x-(-1.65/res))+ (y-(-4.6/res))*(y-(-4.6/res))- ((0.81/res)+d)*((0.81/res)+d))
#        #Capsule
#        u=-3.2516     #x-position of the center
#        v=3.2505    #y-position of the center
#          
#        a=(3.1968-1.599)/2   #radius on the x-axis
#        b=1.599/2    #radius on the y-axis
#        r = [u-a, u+a,u+a, u-a]
#        s = [v-b, v-b, v+b,v+b]
#        
#        u1=u-a
#        u2=u+a
#        e1= ((x-(u1/res))*(x-(u1/res))+ (y-(v/res))*(y-(v/res)) - ((b/res)+d)*((b/res)+d))
#        e2= ((x-(u2/res))*(x-(u2/res))+ (y-(v/res))*(y-(v/res)) - ((b/res)+d)*((b/res)+d))
        exist=True
        if (x>=(-12.5)+d and x<=(12.5/res)-d and y>=(-7.5/res)+d and y<=(7.5/res)-d):
            for c in obstacleList[0]:
                
                   if( x>=c[0][0]-d and x<=c[0][1]+d and y>=c[1][0]-d and y<=c[1][2]+d):
#                       print('1')
                       exist = False 
            if(exist is True):
                for j in circ:
                    if(j<=0):
                        exist=False
#                if( x>=((r[0]/res)-d) and x<=((r[1]/res)+d) and y>=((s[0]/res)-d) and y<=((s[2]/res)+d)):
#                           exist = False
#                           print('2')
#                elif (e1<=0):
#                    exist=False
#                    print('3')
#                elif (e2<=0):
#                    exist=False 
#                elif (c1<=0):
#                    exist=False
#                elif (c2<=0):
#                    exist=False
#                elif (c3<=0):
#                    exist=False
#                elif (c4<=0):
#                    exist=False  
#                else:
#                    exist=True
               
        else:
            exist=False
            
        return exist


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

def getxs_ys(xs,ys,n):
    radius=0
#    global resolution,radius,clearance,init,final
    t = np.linspace(0, 2*np.pi, 100)
#    res=resolution
    resolution=1
#    n=20
    r=[]
    x=[]
    y=[]
    p=[]
    q=[]
#    #Circles
    for i in range(n):
        rad=random.uniform(0.1,0.2)
        x1=random.uniform(-12.5,12.5)
        y1=random.uniform(-7.5,7.5)
        r.append(rad)
        x.append(x1) 
        y.append(y1) 
        p.append(x1+rad*np.cos(t))
        q.append(y1+rad*np.sin(t))
#        plt.fill((x1+rad*np.cos(t)),(y1+rad*np.sin(t)),'r',edgecolor='b')
    circles=[r,x,y,p,q]
#    print(r,x,y)
#    #Circle 1
#    r1 = (0.81/2)/resolution
#    n1=-1.65/resolution    #x-position of the center
#    m1=4.6/resolution   #radius on the y-axis
#    p1=n1+r1*np.cos(t)
#    q1=m1+r1*np.sin(t)
#    for i in p1:
#        xs.append(i)
#    for i in q1:
#        ys.append(i)
#    #Circle 2
#    r2 =( 0.81/2)/resolution
#    n2=-1.17 /resolution    #x-position of the center
#    m2=2.31/resolution   #radius on the y-axis
#    p2=n2+r2*np.cos(t)
#    q2=m2+r2*np.sin(t)
#    for i in p2:
#        xs.append(i)
#    for i in q2:
#        ys.append(i)
#    #Circle 3
#    r3 = (0.81/2)/resolution
#    n3=-1.17/resolution    #x-position of the center
#    m3=-2.31/resolution   #radius on the y-axis
#    p3=n3+r3*np.cos(t)
#    q3=m3+r3*np.sin(t)
#    for i in p3:
#        xs.append(i)
#    for i in q3:
#        ys.append(i)
#    #Circle 4
#    r4 = (0.81/2)/resolution
#    n4=-1.65 /resolution    #x-position of the center
#    m4=-4.6 /resolution  #radius on the y-axis
#    p4=n4+r4*np.cos(t)
#    q4=m4+r4*np.sin(t)
#    for i in p4:
#        xs.append(i)
#    for i in q4:
#        ys.append(i)
#    #Capsule
#    u=-3.2516/resolution     #x-position of the center
#    v=3.2505/resolution    #y-position of the center
#      
#    a=(((3.1968/resolution)-(1.599/resolution))/2)   #radius on the x-axis
#    b=(1.599/2)/resolution    #radius on the y-axis
#    r = [u-a, u+a,u+a, u-a]
#    s = [v-b, v-b, v+b,v+b]
#    for i in r:
#        xs.append(i)
#    for i in s:
#        ys.append(i)
#    u1=u-a
#    u2=u+a
#    r1=u1+b*np.cos(t)
#    s1=v+b*np.sin(t)
#    r2=u2+b*np.cos(t)
#    s2=v+b*np.sin(t)
#    for i in r1:
#        xs.append(i)
#    for i in s1:
#        ys.append(i)
#    for i in r2:
#        xs.append(i)
#    for i in s2:
#        ys.append(i)
#    #Rectangles
    rectangles =[[-11.5,6,0.1,3],[-10.5,0.5,0.1,6],[-7,4,0.1,7],
    [-3.7,-1,1.5,0.1],[-4.5,-1,0.1,3],[0.45,5.5,4,0.1],
    [2.5,5.5,0.1,4],[7.5,5,0.1,5],[11,4.5,3,0.1],
    [11.25,-1.5,2.5,0.1],[9,-4.575,2,0.1],[9.95,-2.5,0.1,4],
    [5,-6,0.1,3],[4.3,-4.45,1.5,0.1],[-3,-6.5,0.1,2],
    [-1.55,-5.45,3,0.1],[0,-1.5,0.1,8],[1.55,-1.5,3,0.1],
    [-1.3,2.5,2.5,0.1],[-2.5,3.55,0.1,2],[4.05,0.5,2,0.1],
    [3,-0.45,0.1,2],[5,2.05,0.1,3],[-10.5,3.55,4,0.1],
    [-9.5,-5.25,0.1,1.5],[-7.5,-3.5,0.1,2],[-9.5,-6.05,2,0.1],
    [-9.55,-4.45,4,0.1],[-9.05,-2.55,3,0.1],[-7.7,0.45,1.5,0.1],[-8.54,4.5,0.1,2]]
    for i in range(len(rectangles)):
        for j in range(4):
            rectangles[i][j]=rectangles[i][j]/resolution

#    fig, ax = plt.subplots()
##    
##
#    ax.fill(r,s,'r',edgecolor='b')
#    ax.fill(r1,s1,'r')
#    ax.fill(r2,s2,'r')
#    ax.fill(p1,q1,'r',edgecolor='b')
#    ax.fill(p2,q2,'r',edgecolor='b')
#    ax.fill(p3,q3,'r',edgecolor='b')
#    ax.fill(p4,q4,'r',edgecolor='b')
    # ax.fill(uelpx, uelpy,'b')
    rectangle_corner=[]
    for i in (rectangles):

              x = [i[0]-(i[2]/2), i[0]+(i[2]/2),i[0]+(i[2]/2), i[0]-(i[2]/2)]
              y = [i[1]-(i[3]/2), i[1]-(i[3]/2), i[1]+(i[3]/2),i[1]+(i[3]/2)]
              for j in x:
                  xs.append(j)
              for j in y:
                  ys.append(j)
              rectangle_corner.append([x,y])
#              ax.fill(x, y,'r',edgecolor='b')
#    ucir1x=[]
#    ucir1y=[]
#    for i in range(len(p1)):
#        ucir1x.append(p1[i]+radius*np.cos(t))
#        ucir1y.append(q1[i]+radius*np.sin(t))
#    ucir2x=[]
#    ucir2y=[]
#    for i in range(len(p2)):
#        ucir2x.append(p2[i]+radius*np.cos(t))
#        ucir2y.append(q2[i]+radius*np.sin(t))
#    ucir3x=[]
#    ucir3y=[]
#    for i in range(len(p3)):
#        ucir3x.append(p3[i]+radius*np.cos(t))
#        ucir3y.append(q3[i]+radius*np.sin(t))
#    ucir4x=[]
#    ucir4y=[]
#    for i in range(len(p4)):
#        ucir4x.append(p4[i]+radius*np.cos(t))
#        ucir4y.append(q4[i]+radius*np.sin(t))
#    ucap1x=[]
#    ucap1y=[]
#    for i in range(len(r1)):
#        ucap1x.append(r1[i]+radius*np.cos(t))
#        ucap1y.append(s1[i]+radius*np.sin(t))
#    ucap2x=[]
#    ucap2y=[]
#    for i in range(len(r2)):
#        ucap2x.append(r2[i]+radius*np.cos(t))
#        ucap2y.append(s2[i]+radius*np.sin(t))
#    uboxx=[]
#    uboxy=[]
#    for i in range(4):
#        uboxx.append(r[i]+radius*np.cos(t))
#        uboxy.append(s[i]+radius*np.sin(t) )
    urecBoxes=[]
    
    for i in rectangle_corner:
        
        uboxrx=[] 
        uboxry=[]
        
        for j in range(4):
            
            uboxrx.append(i[0][j]+radius*np.cos(t))
            uboxry.append(i[1][j]+radius*np.sin(t) )
        urecBoxes.append([uboxrx,uboxry])

   
        return rectangle_corner,circles,[[urecBoxes,'b'],[rectangle_corner,'r']]
    else:
        return "Please enter both Initial and Final Points",[],[]

def main():
    start = time.time()
    print("Start " + __file__)

    # ====Search Path with RRT====
    ox, oy = [], []
    n=10
    rect_corners,circles,rectangles=getxs_ys(ox,oy,n)
    
#    obstacleList = [
##        (5, 5, 1),
##        (5, 5.5, 1),
##        (5, 6, 1)
##        ,
##        (3, 6, 2),
##        (3, 8, 2),
##        (3, 10, 2),
##        (7, 5, 2),
##        (9, 5, 2)
#    ]  # [x,y,size(radius)]
#    for i in range(4):
##        for j in ([2]):
##            obstacleList.append((0+(i/j),4,0.5))
#        obstacleList.append((0+(i),4,0.5))
#    for i in range(2):
##        for j in ([2]):
##            obstacleList.append((0+(i/j),8,0.5))
#        obstacleList.append((0+(i),8,0.5))
#        obstacleList.append((0+(i+0.5),8,0.5))
#    for i in range(2):
##        for j in ([2]):
##            obstacleList.append((3+(i/j),6,0.5))
#        obstacleList.append((3+(i),6,0.5))
#        obstacleList.append((3+(i+0.5),6,0.5))
#    for i in range(9):
##        for j in ([2]):
##            obstacleList.append((4,15-(i/j),0.5))
#        obstacleList.append((4,15-i,0.5))
#        obstacleList.append((4,15-(i+0.5),0.5))
#    for i in range(6):
##        for j in ([2]):
##            obstacleList.append((8,15-(i/j),0.5))
#        obstacleList.append((8,15-i,0.5))
#        obstacleList.append((8,15-(i+0.5),0.5))
#    for i in range(6):
##        for j in ([2]):
##            obstacleList.append((8,0+(i/j),0.5))
#        obstacleList.append((8,0+i,0.5))
#        obstacleList.append((8,0+i+0.5,0.5))
#    for i in range(4):
##        for j in ([2]):
##            obstacleList.append((25-(i/j),7,0.5))
#        obstacleList.append((25-(i),7,0.5))
#        obstacleList.append((25-(i+0.5),7,0.5))
#    # Set Initial parameters
    rrt = RRT(start=[-9,-1], goal=[1.5,6.8],
              randArea=[[-12.5, 12.5],[-7.5,7.5]], obstacleList=[rect_corners,circles])
    path = rrt.Planning(n,circles,rectangles,animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        print('Path: ',path)
        # Draw final path
        if show_animation:
            rrt.DrawGraph(n,circles,rectangles)
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
    end = time.time()
    print('Execution time - ', end - start)

if __name__ == '__main__':
    main()