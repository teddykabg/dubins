#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Teddy Gyabaah}
# {student id}
# {gyabaah@kth.se}

from dubins import *
import numpy as np
import math
class node :
        def __init__(self,theta,x,y,controls,times,cost) :
            self.theta = theta
            self.x = x 
            self.y = y
            self.controls = controls
            self.times = times
            self.cost = cost

class waypoint :
        def __init__(self,x,y,r) :
            self.x = x 
            self.y = y
            self.r = r

def check_outbounds(x,y,car) :
    if (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub ):
        return False
    return True

def euclidean_distance(x1,y1,x2,y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def check_collisions(x,y,car):
    for obs in car.obs:
        d = euclidean_distance(x,y,obs[0],obs[1])
        if d <= obs[2] + 0.1:
            return True
    return False

def position(car,node_,phi) :
    dt = 0.01
    cost = 0
    threshold = 0.2
    for i in range(100 if phi == 0 else 157):
        x, y, theta = step(car,node_.x, node_.y, node_.theta, phi)
        while theta >= np.pi:
            theta -= 2*np.pi
        while theta <= -2*np.pi:
            theta += np.pi  
        node_.controls.append(phi)
        node_.times.append(node_.times[-1] + dt)
        
        if check_collisions(x,y,car) or check_outbounds(x,y,car):
            node_return = node(0,0,0,node_.controls,node_.times,np.inf)
            return False,node_return
        if euclidean_distance(x,y,car.xt,car.yt) <= threshold :
            node_return = node(theta,x,y,node_.controls,node_.times,0)
            return True, node_return

    cost = euclidean_distance(node_.x,node_.y,car.xt,car.yt)
    ret = node(node_.theta,node_.x,node_.y,node_.controls,node_.times,cost)
    return True, ret

def plan_path_(car, path , visited) :
    #Create open_set with car position
    new_node = node(0,car.x0,car.y0,[],[0],euclidean_distance(car.x0,car.y0,car.xt,car.yt))
    open_set = [new_node]
    C_set = []
    threshold = 0.2
    while len(open_set) > 0 :
        first_node = open_set.pop(0)
        print("The nodeee:")
        print(first_node.controls)
        if euclidean_distance(first_node.x,first_node.y,car.xt,car.yt) <= threshold :
            return first_node.controls, first_node.times
        visited.append([round(first_node.x,1), round(first_node.y,1)])
        for phi in [-np.pi/4, 0, np.pi/4] :
            flag,node_ = position(car,first_node,phi)
            if flag and not [round(node_.x,1), round(node_.y,1), round(node_.theta,1)] in C_set:
                C_set.append([round(node_.x,1), round(node_.y,1), round(node_.theta,1)])
                open_set.append(node_)
            open_set.sort(key=lambda x: x.cost)
    print("No goal!")
    return [],[0] 

    if (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub ):
        return False
    return True
   
def positions(x,y,phi,theta,car,controls,times,threshold):
    cost = 0
    dt = 0.01
    for i in range(100 if phi == 0 else 157):
        x, y, theta = step(car,x, y, theta, phi)
        while theta >= math.pi:
            theta -= 2*math.pi
        while theta <= -2*math.pi:
            theta += math.pi  
        controls.append(phi)
        times.append(times[-1] + dt)
        if check_collisions(x,y,car) or check_outbounds(x,y,car):
            return False, 0, 0, 0, controls, times, np.inf
        if math.sqrt((x - car.xt)**2 + (y - car.yt)**2) <= threshold:
            return True, x, y, theta, controls, times, 0
    cost = math.sqrt((x - car.xt)**2 + (y - car.yt)**2)
    return True, x, y, theta, controls, times, cost

def plan_path(car, path, visited):
    threshold = 0.2
    new_node = node(0,car.x0,car.y0,[],[0],euclidean_distance(car.x0,car.y0,car.xt,car.yt))
    
    open_set = [new_node]
    C_set = []

    queue = [[car.x0,car.y0,0,[],[0],euclidean_distance(car.x0,car.y0,car.xt,car.yt)]]
    queue1 = []
    while len(queue) > 0:
        x,y,theta,controls,times,_ = queue.pop(0)
        #first_node = open_set.pop(0)
        if euclidean_distance(x,y,car.xt,car.yt) <= threshold:
            return controls, times
        visited.append([round(x,1), round(y,1)])
        for phi in [-math.pi/4, 0, math.pi/4]:
            useable, x1, y1, theta1, controls1, times1, cost = positions(x,y,phi,theta,car,replace_array(controls),replace_array(times),threshold)
            if useable and not [round(x1,1), round(y1,1), round(theta1,1)] in queue1:
                path.append(phi)
                queue1.append([round(x1,1), round(y1,1), round(theta1,1)])
                queue.append([x1, y1, theta1, controls1, times1, cost])
            queue.sort(key=lambda x: x[5])
    return [],[0]

def replace_array(arr):
    new_arr = []
    for x in arr:
        new_arr.append(x)
    return new_arr

def solution(car):

    ''' <<< write your code below >>> '''
    #assemble path
    controls=[0]
    times=[0,1]
    
    ''' <<< write your code above >>> '''
    controls, times = plan_path(car, [], [])
   
    #controls,times = plan_path_(car,[],[])

    return controls, times
