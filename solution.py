#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Teddy Gyabaah}
# {student id}
# {gyabaah@kth.se}

from dubins import *
import numpy as np

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

def change_list(lis):
    new_lis = []
    for x in lis:
        new_lis.append(x)
    return new_lis

def check_outbounds(x,y,car) :
    return not (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub )

def euclidean_distance(x1,y1,x2,y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def check_collisions(x,y,car):
    for obs in car.obs:
        if euclidean_distance(x,y,obs[0],obs[1]) <= obs[2] + 0.1:
            return True
    return False

def is_in_cset(x,y,theta,c_set):
    return [round(x,1), round(y,1), round(theta,1)] in c_set

def try_steering_angles(range_,first_node,car,open_set,c_set):
    for phi in range_ :
            flag,node_ = calculate_positions(first_node.x,first_node.y,phi,first_node.theta,car,change_list(first_node.controls),change_list(first_node.times),0.3)
            if flag and not is_in_cset(node_.x,node_.y,node_.theta,c_set) :
                c_set.append([round(node_.x,1), round(node_.y,1), round(node_.theta,1)])
                open_set.append(node_)
            open_set.sort(key=lambda x: x.cost)

def calculate_positions(x,y,phi,theta,car,controls,times,threshold):
    cost = 0
    dt = 0.01
    iterations = 0

    if phi == 0 :
        iterations = 100
    else :
        iterations = 157

    for i in range(iterations):
        x, y, theta = step(car,x, y, theta, phi)
        while theta >= np.pi:
            theta -= 2*np.pi

        while theta <= -2*np.pi:
            theta += np.pi  

        controls.append(phi)
        times.append(times[-1] + dt)
        #Chechk whether the path is not to follow, in that case we attribute high cost
        if check_collisions(x,y,car) or check_outbounds(x,y,car):
            node_return = node(0,0,0,controls,times,10000)
            return False, node_return
        #If I'm at the final destination lowest cost possible 0s
        elif euclidean_distance(x,y,car.xt,car.yt) <= threshold:
            node_return = node(theta,x,y,controls,times,0)
            return True, node_return

    cost = euclidean_distance(x,y,car.xt,car.yt)
    node_return = node(theta,x,y,controls,times,cost)
    return True, node_return

def plan_path(car):
    #Implementation based on BFS algorithm
    threshold = 0.3
    angle_range = [-np.pi/4, 0, np.pi/4] 
    new_node = node(0,car.x0,car.y0,[],[0],euclidean_distance(car.x0,car.y0,car.xt,car.yt))
    open_set =[new_node]
    c_set = []

    while len(open_set) > 0:
        first_node = open_set.pop(0)
        #If the condition is satisfied I reached the target
        if euclidean_distance(first_node.x,first_node.y,car.xt,car.yt) <= threshold :
            return first_node.controls, first_node.times
        #Computes and evaluate possible paths given different steering angles
        try_steering_angles(angle_range,first_node,car,open_set,c_set)
    return [],[0]



def solution(car):

    return  plan_path(car)
