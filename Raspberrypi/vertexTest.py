# -*- coding: utf-8 -*-
import math
import urllib
import json
pi = 3.141592653589793


class Vertex(object):
    
    def __init__(self, name, node_id, x, y):
        """ initializes a vertex object """
        self.name = name    #the name of vertice
        self.id = node_id   #the id of vertice
        self.x = x          #the position of vertice on X axis
        self.y = y          #the position of vertice on Y axis
        
    def __str__(self):
        res = str(self.name) + "(" + str(self.x) + "," + str(self.y) + ")"
        return res


    
    #Add other methods below
    #This function will return the distance between two vertices
    def distance(self,end_x,end_y):
        dis = math.sqrt(math.pow((self.x-end_x),2) + math.pow((self.y-end_y),2))
        return dis

    #This function will calculate the relative angle between two vertices
    def rel_angle(self,end_x,end_y, north_dir):

        rel_vec_x = end_x - self.x
        rel_vec_y = end_y - self.y
        ref_x = 0
        ref_y = 1

        dianji = ref_x * rel_vec_x + ref_y * rel_vec_y
        moji = math.sqrt(math.pow(ref_x,2) + math.pow(ref_y,2)) * math.sqrt(math.pow(rel_vec_x,2) + math.pow(rel_vec_y,2))
        if moji == 0:
            return 0
        else:
            if rel_vec_x < 0:
                #return -math.acos(dianji/moji)  #rad
                return  - math.acos(dianji/moji)/pi*180 - north_dir     #degree
            else:
                #return math.acos(dianji/moji)  #rad
                return  math.acos(dianji/moji)/pi*180 - north_dir       #degree


