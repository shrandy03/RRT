import random
import math
import matplotlib.pyplot as plt
import numpy as np

class rrt:
    class node:
        def __init__(self,x,y):
            self.x = x
            self.y = y
            self.parent = None
            # for the edges
            self.p_x = []
            self.p_y = []
    
    def __init__(self,start, goal,num_obs,obs_list,edge_pace,max_iter,maxw,maxh,step_size,sample_rate = 5):
        self.start = self.node(start[0],start[1])
        self.goal = self.node(goal[0],goal[1])
        self.num_obs = num_obs
        self.obs_list = obs_list
        self.edge_pace = edge_pace
        self.max_iter = max_iter
        self.node_list = None
        self.maxw = maxw
        self.maxh = maxh
        self.step_size = step_size
        self.sample_rate = sample_rate

    # for path planning 

    def path(self):

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_nod = self.rnd_node()
            near_ind = self.near_in(self.node_list,rnd_nod)
            near_node = self.node_list[near_ind]
            d,t = self.dist_ang(near_node,rnd_nod)
            new_node = self.create_edge(near_node,rnd_nod,d,t,)

            if (self.check_coll(self.obs_list,new_node)):
                self.node_list.append(new_node)

            if i%5 :
                self.draw(rnd_nod)
            
            final_d = math.sqrt((self.node_list[-1].x - self.goal.x)**2 + (self.node_list[-1].y - self.goal.y)**2)
            if(final_d)<= 3.0 :
                d,t = self.dist_ang(self.node_list[-1],self.goal)
                final_n = self.create_edge(self.node_list[-1],self.goal,d,t)
                
                if(self.check_coll(self.obs_list,final_n)):
                    return self.final_path()
            
            
            
        return None


    def rnd_node(self):
        rnd = self.node(random.uniform(self.maxw,self.maxh),random.uniform(self.maxw,self.maxh))
        return rnd
    
    def dist_ang(self,from_n,to_n):
        d = math.sqrt((from_n.x - to_n.x)**2 + (from_n.y - to_n.y)**2)
        t = math.atan2((from_n.y - to_n.y), (from_n.x - to_n.x))
        a = [d,t]
        return a
    
    def create_edge(self,from_n,to_n,d,t):
        new = self.node(from_n.x,from_n.y)
        new.p_x = [new.x]
        new.p_y = [new.y]
        step = self.step_size
        pace = self.edge_pace
        if(step >d):
            step = d
        
        n = math.floor(step/pace)

        for i in range(n):
            new.x += (pace*math.cos(t))
            new.y += (pace*math.sin(t))
            new.p_x.append(new.x)
            new.p_y.append(new.y)
        
        D,T = self.dist_ang(new,to_n)
        if D<= pace :
            new.p_x.append(to_n.x)
            new.p_y.append(to_n.y)
            new.x = to_n.x
            new.y = to_n.y
        
        new.parent = from_n
        return new
    
    def final_path(self):
        l = len(self.node_list) - 1
        final_c = [[self.goal.x,self.goal.y]]
        node = self.node_list[l]
        while node.parent != None:
            final_c.append([node.x,node.y])
            node = node.parent
        final_c.append([node.x,node.y])
        return final_c
    
    # to visualize the code
    def draw(self,rnd_node = None):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
        
        if rnd_node != None:
            plt.plot(rnd_node.x, rnd_node.y, "^k")
        
        for i in self.node_list:
            if i.parent:
                plt.plot(i.x,i.y,"-g")
        
        for (x,y,s) in self.obs_list:
            self.plot_circle(x, y, s)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)  

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def near_in(node_list,rnd):
        node_dis = [ math.sqrt((rnd.x - i.x)**2 + (rnd.y - i.y)**2) for i in node_list]
        min_d = node_dis.index(min(node_dis))
        return min_d  
    
    @staticmethod
    def check_coll(obs_list,new_n):
        if new_n is None:
            return False
        
        for (x,y,s) in obs_list:
                dx = [ (x - i)**2  for i in new_n.p_x]
                dy = [ (y - i)**2  for i in new_n.p_y]
                d = [ math.sqrt(d_x + d_y) for (d_x,d_y) in zip(dx,dy) ]
                if min(d) <= s :
                    return False
        
        return True


obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]

RRT = rrt(start = [0,0], goal = [6.0,10.0] , num_obs = 7 , obs_list = obstacleList, edge_pace=0.5, max_iter=500,maxw = -2,maxh=15,step_size=3.0)

p = RRT.path()

if p is None:
    print("None")
else :
    print("found path!!")
    RRT.draw()
    plt.plot([x for (x, y) in p], [y for (x, y) in p], '-r')
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show()
    print()
