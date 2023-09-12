import matplotlib.pyplot as plt
from rrt import *
import random
import numpy as np
# import matplotlib.patches as shape

fig, ax = plt.subplots()

plt.xlim([0, 100])
plt.ylim([0, 100])
fig.canvas.draw() 

def distance(point1,point2):
    a=np.array(point1)
    b=np.array(point2)
    return np.linalg.norm(a-b)


num_obs=10
goal=[(random.random()*100)-1,(random.random()*100)-1]
# goal=[8,98]
start=[(random.random()*100)-1,(random.random()*100)-1]
# start=[76,14]
while distance(start,goal)<50:
    start=[(random.random()*100)-1,(random.random()*100)-1]

D=[]
obs=0
while obs!=num_obs:
    center=[(random.random()*100)-1,(random.random()*100)-1]
    radius=(random.random()*10)-1
    if distance(center,start)>radius and distance(center,goal)>radius:
        D.append([center,radius])
        obs+=1


print(start,goal)


K=1000
algo=RRT(start,K,D,1,goal)

graph,solved,last_node=algo.run()
print('got it')
for circle in D:
    Drawing_colored_circle=plt.Circle((circle[0][0],circle[0][1]),circle[1])
    ax.set_aspect( 1 )
    ax.add_artist( Drawing_colored_circle )
    


for node in graph:
    
    if node.parent:
        
        point1=node.positon
        point2=node.parent.positon
        # print(point1,point2)
        x_values = [point1[0], point2[0]]
        y_values = [point1[1], point2[1]]
        plt.plot(x_values, y_values, 'bo-')
        

if solved:
    print('stuck 1')
    
    while last_node.parent!= None:
        point1=last_node.positon
        point2=last_node.parent.positon
        # print(point1,point2)
        print(point1,point2)
        x_values = [point1[0], point2[0]]
        y_values = [point1[1], point2[1]]
        print('stuck 2')
        plt.plot(x_values, y_values, 'ro-')
        last_node=last_node.parent

plt.show()


