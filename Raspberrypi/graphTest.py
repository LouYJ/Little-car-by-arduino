#learn how to import your own code
#need to import vertexTest.py in some ways
from vertexTest import Vertex
from jsonTest import grab_map
import math
import urllib
import json
from os import system
#from espeak import espeak

class Graph(object):

    def __init__(self):
        """ initializes a graph object """
        """ figure out what to store internally for a graph"""
        self.E = [] #the map of edges and weights
        self.V = [] #the list of vertices 
        self.nodenum = 0 #the number of vertices
        self.flag = 0 #flag of error
        self.northAt = 0.0  #the north direction
        self.start_id = 0
        self_end_id = 0
        self.pid_banned = [] # [15, 32, 39, 17, 21, 24, 26]  # com1 id that does not need pid

        if None == self.Build_Graph():
            self.flag = 1
            # if this flag is set to be 1, there is an error when building map.

    '''
    This function will build the graph by calling the jsonTest related fucntions
    '''
    def Build_Graph(self):
        #get the initial information from URL
        myJson = grab_map("DemoBuilding", "1")  #for initialization

        result = myJson.execute() #get input from user and download/get the map from cache
        if (myJson.building == "1" and myJson.level == "2"):
            self.pid_banned = [15, 32, 39, 17, 21, 24, 26]
            
        map_info = ""
        if (result == "success"):
            map_info = myJson.info
        else:
            return None
        urlmap = map_info.get('map')    #retrieve the map from general information
        self.northAt = float(map_info.get('info')["northAt"])  #get the north direction provided by general information
        #get all vertices 
        print('The number of vetices of map is: ' + str(len(urlmap)) )
        #espeak.synth(str(len(urlmap)) + 'vertices in map')
        num = len(urlmap)
        index1 = 0
        nodes = []
        while index1 < num:
            # print('num is: ' + str(index1))
            nodes.append(Vertex(urlmap[index1]['nodeName'], urlmap[index1]['nodeId'], int(urlmap[index1]['x']), int(urlmap[index1]['y'])))
            index1 = index1 + 1

        #output all vertices 
        index1 = 0
        if num > 0:
            '''
            while index1 < num:
                print(nodes[index1])
                index1 = index1 +1
            '''
            print(nodes[index1])
            
        '''get the list and matrix'''
        graph_matrix = [[-1 for i in range(num)] for i in range(num)] 

        index1 = 0
        while index1 < num:
            target_sum = urlmap[index1]['linkTo']
            str_target = str(target_sum)
            targets = str_target.split(",")
            for target in targets:
                #print(str(index1+1) + ' links to: '+ target)
                graph_matrix[index1][int(target)-1] = nodes[index1].distance(nodes[int(target)-1].x,nodes[int(target)-1].y)
                graph_matrix[int(target)-1][index1] = nodes[index1].distance(nodes[int(target)-1].x,nodes[int(target)-1].y)
            
            index1 = index1 + 1
        
        index1 = 0

        self.V=nodes
        self.E=graph_matrix
        self.nodenum = num 

        return "success"

    '''
    This function will output the entire graph
    '''
    def Print_Graph(self):
        graph_matrix = self.E
        num = self.nodenum
        index1 = 0
        print('------------------------------')
        while index1 < num:
            print(graph_matrix[index1])
            index1 = index1 + 1

    '''
    This function will add new vertices to the graph
    '''       
    def AddVertex(self,v,links):
        self.V.append(v)
        nodes = self.V
        graph_matrix = self.E
        num = self.nodenum + 1

        graph_matrix = [[-1 for i in range(num)] for i in range(num)] 
        i = 0
        j = 0

        #restore the old information of graph
        for i in range (0,self.nodenum):
            for j in range  (0,self.nodenum):
                graph_matrix[i][j] = self.E[i][j]
                #print(i,j)
        index1 = int(v.id)-1

        print('add id is: ' + str(index1))
        # add 1 line and 1 row
        for target in links:
            print(str(target) + ' links to: '+ str(index1+1))
            graph_matrix[index1][int(target)-1] = nodes[index1].distance(nodes[int(target)-1].x,nodes[int(target)-1].y)
            graph_matrix[int(target)-1][index1] = nodes[index1].distance(nodes[int(target)-1].x,nodes[int(target)-1].y)

        #store new map
        self.nodenum = num
        self.E = graph_matrix
        self.Print_Graph()
