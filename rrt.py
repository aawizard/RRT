import random
import numpy as np


class Node:
    def __init__(self,pos,dis,parent) :
        self.positon=pos
        self.distance=dis
        self.parent=parent
        




class RRT:
    def __init__(self,init_pos, K, D,inc_dis,goal):
        self.q_init=Node(init_pos,0,None)
        self.K=K
        self.delta=inc_dis
        self.D=D
        self.G=[self.q_init]
        self.goal=goal 


    def check_way(self,point1,point2):
        """_summary_

        Args:
            point1 (_type_): _description_
            point2 (_type_): _description_

        Returns:
            _False: there is collision
            True: if point is posssible
        """
        p1=np.array(point1)
        p2=np.array(point2)
        
        for obs in self.D:
            p3=np.array(obs[0])
            
            dis=np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
            
            if dis<obs[1]:
                print(dis,obs[1])
                return False
        return True

    
    def check_direct(self,point):
        if self.check_way(point,self.goal):
            return True
        else:
            return False
        
        
    
    def generate_random(self):
        return [random.random()*100-1,random.random()*100-1]
    
    
    def distance(self,point1,point2):
        a=np.array(point1)
        b=np.array(point2)
        return np.linalg.norm(a-b)
        
    def get_closest_node(self,point):
        min_dis_node=self.G[0]
        # print(p,point)
        min_dis= 1000000
        
        
        for p in self.G:
            dis=self.distance(p.positon,point)
            # print(p.positon)
            if dis<min_dis and self.check_way(p.positon,point):
                min_dis=dis
                min_dis_node=p
                # print("hi")
                
                
        return min_dis_node,min_dis
    
    
    def get_node(self,point):
        closest_node,dist=self.get_closest_node(point)
        if dist==1000000:
            return False
        else:
            if dist<=self.delta:
                new_node=Node(point,dist,closest_node)
                
            else:
                a=np.array(closest_node.positon)
                b=np.array(point)
                
                new_point_array=(((b-a)/dist)*self.delta)+a
                new_point=[new_point_array[0],new_point_array[1]]
                new_node=Node(new_point,self.delta,closest_node)
            
            return new_node
    
        

    
    def run(self):
        for step in range(self.K):
            # print(step)
            point=self.generate_random()
            new_node=self.get_node(point)
            if new_node:
                self.G.append(new_node)
                # print('here')
                if self.check_direct(new_node.positon):
                    # print('here2')
                    last_node=Node(self.goal,self.distance(self.goal,new_node.positon),new_node)
                    # print('here3')
                    self.G.append(last_node )
                    return self.G,True,last_node

                
        return self.G,False,None
    
    



















