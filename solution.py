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
    return not (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub )

def euclidean_distance(x1,y1,x2,y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def check_collisions(x,y,car):
    for obs in car.obs:
        if euclidean_distance(x,y,obs[0],obs[1]) <= obs[2] + 0.1:
            return True
    return False

def positions_(x,y,phi,theta,car,controls,times,threshold):
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
            node_return = node(0,0,0,controls,times,np.inf)
            return False, node_return
        if euclidean_distance(x,y,car.xt,car.yt) <= threshold:
            node_return = node(theta,x,y,controls,times,0)
            return True, node_return
    cost = euclidean_distance(x,y,car.xt,car.yt)
    node_return = node(theta,x,y,controls,times,cost)
    return True, node_return

def plan_path(car):
    threshold = 0.2
    angle_range = [-math.pi/4, 0, math.pi/4]
    new_node = node(0,car.x0,car.y0,[],[0],euclidean_distance(car.x0,car.y0,car.xt,car.yt))
    open_set =[new_node]
    c_set = []
    visited = []

    while len(open_set) > 0:
        first_node = open_set.pop(0)
        if euclidean_distance(first_node.x,first_node.y,car.xt,car.yt) <= threshold :
            return first_node.controls, first_node.times
        visited.append([round(first_node.x,1), round(first_node.y,1)])
        for phi in angle_range :
            flag,node_ = positions_(first_node.x,first_node.y,phi,first_node.theta,car,replace_array(first_node.controls),replace_array(first_node.times),threshold)
            if flag and not [round(node_.x,1), round(node_.y,1), round(node_.theta,1)] in c_set:
                c_set.append([round(node_.x,1), round(node_.y,1), round(node_.theta,1)])
                open_set.append(node_)
            open_set.sort(key=lambda x: x.cost)
    return [],[0]

def replace_array(arr):
    new_arr = []
    for x in arr:
        new_arr.append(x)
    return new_arr

def solution(car):

    return  plan_path(car)
