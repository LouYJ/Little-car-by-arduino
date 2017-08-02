# -*- coding: utf-8 -*-
from vertexTest import Vertex
import math
import urllib
import json
from graphTest import Graph
from dijkstra import dijkstra
#from #espeak import #espeak 
import cal_position

def navigation():

	#build the graph according to the URL provided bu user
	new_graph = Graph()
	#espeak.set_parameter(#espeak.Parameter.Rate, 200)
	#espeak.set_parameter(#espeak.Parameter.Pitch, 60)
	#espeak.set_parameter(#espeak.Parameter.Range, 700)

	reset_flag = 0 #the flag for reset 
	reset_start = 1 #the new starting point for reset 

	get_stuck = 0 #whether these is an obstacle stop the vehicle keeping moving forward

	if (new_graph.flag == 1):
		print ("No available map.")
		#espeak.synth("No available map.")
		return None
	print('Map is successfully downloaded !')
	#espeak.synth('Map is successfully downloaded !')
	north_dir = float(new_graph.northAt) 
	print ("The north is at " + str( north_dir))

	#new_graph.Print_Graph()
	#get the starting point and the ending point
	start_p = int(input('please input the starting point\'s id: '))
	end_p = int(input('please input the ending point\'s id: '))
	while (start_p > len(new_graph.V) or end_p > len(new_graph.V) or start_p <= 0 or end_p <= 0):
            print('-----invalid input, please input again !-----')
            start_p = int(input('please input the starting point\'s id: '))
            end_p = int(input('please input the ending point\'s id: '))

	#scale the direction from -180 to 180 degrees 
	if north_dir > 180:
		north_dir = north_dir - 360

	#run the dijkstra and find the path (a list of vertices' id
	path = dijkstra(new_graph, int(start_p), int(end_p)) 
	length = len(path)
	#scale vertices' id to the index of the graph_matrix
	print('path is :' + str(path))
	for i in range(0,length):
		path[i] = path[i] - 1
	

	'''
	Now the navigation gets started, and here are comments of important variables
	[cur_pos_x, cur_pos_y] : The current position. The default current position is the starting point
	Keep_dir : The correct direction to next check point, and this should equal to the current direction (cur_dir)
	'''
	index = 0
	cur_pos_x = new_graph.V[path[0]].x
	cur_pos_y = new_graph.V[path[0]].y
	keep_dir = cur_dir = int(input('please input current direction:'))
	#[cur_pos_x, cur_pos_y] = cal_position(cur_pos_x, cur_pos_y, north_dir)  #"cur_dir" and "dis" are got from Arduino

	while (check_point_area(cur_pos_x, cur_pos_y, new_graph.V[path[length-1]]) == 0):

		tmp_point = Vertex('temp', 0, cur_pos_x, cur_pos_y)
	
		if (check_point_area(cur_pos_x, cur_pos_y, new_graph.V[path[index]]) == 1):

			#get the correct direction pointing at the next check point
			index = index + 1
			direction = tmp_point.rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, 
				north_dir)
			#get and output the instruction of current movement 
			keep_dir = move_instruction(direction, cur_dir)
		else:
			reset_start = check_wrong_route(cur_pos_x, cur_pos_y, new_graph, index)
			if (reset_start != index and reset_start != -1):
				reset_start = reset_start + 1
				reset_flag = 1
				print('Wrong route ! Reset route now!')
				#espeak.synth('Wrong route ! Reset route now!')
				if (reset_flag == 1):
					start_p = reset_start
				reset_flag = 0	
				path = dijkstra(new_graph, int(start_p), int(end_p)) 
				length = len(path)
				#scale vertices' id to the index of the graph_matrix
				print('new path is :' + str(path))
				for i in range(0,length):
					path[i] = path[i] - 1

				#get the correct direction pointing at the next check point
				index = 0
				cur_pos_x = new_graph.V[path[0]].x
				cur_pos_y = new_graph.V[path[0]].y
				index = index + 1
				tmp_point = Vertex('temp', 0, cur_pos_x, cur_pos_y)
				direction = tmp_point.rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, 
					north_dir)
				#get and output the instruction of current movement 
				keep_dir = move_instruction(direction, cur_dir)
			elif (reset_start == -1):
				direction = tmp_point.rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, 
					north_dir)
				keep_dir = move_instruction(direction, cur_dir)


		#calculate the distance between current position and the next check point
		cur_distance = math.sqrt(math.pow((cur_pos_x - new_graph.V[path[index]].x),2) 
			+ math.pow((cur_pos_y - new_graph.V[path[index]].y),2))

		#calibrate the current direction to the correct direction
		cur_dir = dir_calibrate(keep_dir, cur_dir)

		#output and get the new position
		print('the curent direction  is: ' + str(round(cur_dir,2)))
		#espeak.synth('curent direction  is: ' + str(round(cur_dir,2)))
		print('the rest distance is: ' + str(round(cur_distance,2)))
		#espeak.synth('rest distance is: ' + str(round(cur_distance,2)) + 'centimeters')
		
		cur_pos_x = int(input('please input current X position:'))
		cur_pos_y = int(input('please input current Y position:'))
		cur_dir = int(input('please input current direction:'))
		#[cur_pos_x, cur_pos_y] = cal_position(cur_pos_x, cur_pos_y, north_dir)  #"cur_dir" and "dis" are got from Arduino
		
	print('You have arrived the at destination !')
	#espeak.synth('You have arrived at the destination !')

def check_wrong_route(cur_pos_x, cur_pos_y, new_graph, index_raw):
	v_length = new_graph.nodenum	#get the number of all vertices 
	v_list = new_graph.V    		#get the list of vertices 
	index_cur = 0
	while index_cur < v_length :
		if (index_cur != index_raw and check_point_area(cur_pos_x, cur_pos_y, v_list[index_cur])):
			return index_cur
		elif (index_cur == index_raw and check_point_area(cur_pos_x, cur_pos_y, v_list[index_cur])):
			return index_raw
		else:
			index_cur = index_cur + 1
	return -1
'''
This function will decide whether the current position is clost to next check point, if error is considered
return 1 ---- the vehicle has entered the checking area
return 0 ---- the vehicle needs to keep moving forward
'''
def check_point_area(cur_pos_x,cur_pos_y, target):
	if (cur_pos_x == target.x and cur_pos_y == target.y):
		return 1
	elif (target.distance(cur_pos_x,cur_pos_y) < 15):
		return 1
	else:
		return 0

'''
This function will provide instructions according to the current situation.
final_direction : the degrees that the current direction needs to change
'''
def move_instruction(direction, cur_dir):
	final_direction = 0
	if direction < 0:
		direction = direction + 360	#rescale the relative direction to 0-360

	final_direction = direction - cur_dir

	if final_direction > 0:
            if final_direction <= 180:
                print('turn right for ' + str(round(final_direction,2)) + ' degrees !')
                #espeak.synth('turn right for ' + str(round(final_direction,2)) + ' degrees !')
            elif final_direction > 180 and final_direction <=360:
                final_direction = final_direction - 360
                print('turn left for ' + str(round(final_direction,2)) + ' degrees !')
                #espeak.synth('turn left for ' + str(round(final_direction,2)) + ' degrees !')
	elif final_direction < 0:
            if (-final_direction) <= 180:
                print('turn left for ' + str(round(final_direction,2)) + ' degrees !')
                #espeak.synth('turn left for ' + str(round(final_direction,2)) + ' degrees !')
            elif (-final_direction)> 180 and (-final_direction)<=360 :
                final_direction = final_direction + 360
                print('turn right for ' + str(round(final_direction,2)) + ' degrees !')
                #espeak.synth('turn right for ' + str(round(final_direction,2)) + ' degrees !')
                
	else:
		print('please go straight forward !')
		#espeak.synth('please go straight!')

	return direction


'''
This function will calibrate the current direction to the correct direction to next check point (keep_dir)
final_direction : the degrees that the current direction needs to change
'''
def dir_calibrate(keep_dir, cur_dir):
	if keep_dir != cur_dir:
		#print('-----calibrate direction-----')
		final_direction = keep_dir - cur_dir
		
		if final_direction > 0:
			print('turn the direction right for ' + str(round(final_direction,2)) + ' degrees !')
			##espeak.synth('turn the direction right for ' + str(final_direction) + ' degrees !')
		elif final_direction < 0:
			print('turn the direction left for ' + str(-round(final_direction,2)) + ' degrees !')
			##espeak.synth('turn the direction left for ' + str(-final_direction) + ' degrees !')
		
		print('-----calibrate direction-----')
		##espeak.synth('the direction has been calibrated')
		return keep_dir
	return keep_dir


if __name__ == '__main__':
	navigation()
