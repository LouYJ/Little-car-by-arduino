# -*- coding: utf-8 -*-
from vertexTest import Vertex
import math
import urllib
import json
from graphTest import Graph
from dijkstra import dijkstra
#from espeak import espeak 
import cal_position
from Handshake import Handshake
from cal_position import cal_position
from cal_position import send_req
from cal_position import get_ins
from cal_position import send_ins
from cal_position import send_terminal
from os import system

def cross_navigation():
        
	#build the graph according to the URL provided bu user


	new_graph = Graph()
	if (new_graph.flag == 1):
		print ("No map.")
		#espeak.synth("No map.")
		return None
	cross_graph = Graph()
	if (cross_graph.flag == 1):
		print ("No available cross map.")
		system("echo no available map | festival --tts")
		#espeak.synth("No cross map.")
		return None
	print('Map is successfully downloaded !')
	system("echo map has been successfully downloaded | festival --tts")

	#espeak.set_parameter(#espeak.Parameter.Rate, 200)
	#espeak.set_parameter(#espeak.Parameter.Pitch, 60)
	#espeak.set_parameter(#espeak.Parameter.Range, 700)
	reset_flag = 0 #the flag for reset 
	reset_start = 1 #the new starting point for reset 
	get_stuck = 0 #whether these is an obstacle stop the vehicle keeping moving forward

	##espeak.synth('Map is successfully downloaded !')
	north_dir = float(new_graph.northAt) 
	print ("The north is at " + str( north_dir))
	cross_north_dir = float(cross_graph.northAt)
	print ("The cross-map north is at " + str( north_dir))
	#begin to handshake
	
	Handshake()
	print('end handshake' )
	#get the starting point and the ending point
	#espeak.synth('starting point')
	system("echo please input the start point in the first building | festival --tts")
	start_p = int(input('please input the starting point\'s id: '))
	#espeak.synth('ending point')
	system("echo end point in the second building| festival --tts")
	end_p = int(input('please input the end point in the second building\'s id: '))
	while (start_p > len(new_graph.V) or start_p <= 0 or end_p <= 0 or end_p > len(cross_graph.V)):
            system("echo invalid input please input again | festival --tts")
            print('-----invalid input, please input again !-----')
            system("echo start point | festival --tts")
            start_p = int(input('please input the starting point\'s id: '))
            system("echo end point | festival --tts")
            end_p = int(input('please input the ending point\'s id: '))

	#scale the direction from -180 to 180 degrees 
	if north_dir > 180:
		north_dir = north_dir - 360
	if cross_north_dir > 180:
		cross_north_dir = cross_north_dir - 360

	#run the dijkstra and find the path (a list of vertices' id
	# vertice 31 is the connection between 2 building
	path = dijkstra(new_graph, int(start_p), int(31)) 
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
	
	ins_flag = 0
	cur_pos_x = 0
	cur_pos_y = 0
	
	first_flag = 0
	send_req()
	cur_pos_x = new_graph.V[path[0]].x
	cur_pos_y = new_graph.V[path[0]].y
	[cur_pos_x, cur_pos_y, cur_dir, ins_flag] = cal_position(cur_pos_x, cur_pos_y, north_dir)  #"cur_dir" and "dis" are got from Arduino
	ins_flag = 1
	ok_cnt = 1
	keep_dir = cur_dir
	first_dir_flag = 1
	while (check_point_area(cur_pos_x, cur_pos_y, new_graph.V[path[length-1]]) == 0 and ok_cnt < length):

		tmp_point = Vertex('temp', 0, cur_pos_x, cur_pos_y)
		index = index + 1
		ok_cnt = ok_cnt + 1
		if first_dir_flag == 1:
			direction = tmp_point.rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, north_dir)
			first_dir_flag = 0
		elif first_dir_flag == 0:
			direction = new_graph.V[path[index-1]].rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, north_dir)
		#direction = tmp_point.rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, north_dir)
		#get and output the instruction of current movement 
		[keep_dir, turn_dir] = move_instruction(direction, cur_dir)
		'''
		if (check_point_area(cur_pos_x, cur_pos_y, new_graph.V[path[index]]) == 1):

			#get the correct direction pointing at the next check point
			index = index + 1
			ok_cnt = ok_cnt + 1
			direction = tmp_point.rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, 
				north_dir)
			#get and output the instruction of current movement 
			[keep_dir, turn_dir] = move_instruction(direction, cur_dir)
		else:
			reset_start = check_wrong_route(cur_pos_x, cur_pos_y, new_graph, index)
			if (reset_start != index and reset_start != -1):
				reset_start = reset_start + 1
				reset_flag = 1
				print('Wrong route ! Reset route now!')
				#espeak.synth('Wrong route ! ')
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
				ok_cnt = ok_cnt + 1
				tmp_point = Vertex('temp', 0, cur_pos_x, cur_pos_y)
				direction = tmp_point.rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, 
					north_dir)
				#get and output the instruction of current movement 
				[keep_dir, turn_dir] = move_instruction(direction, cur_dir)
			elif (reset_start == -1):
				direction = tmp_point.rel_angle(new_graph.V[path[index]].x, new_graph.V[path[index]].y, 
					north_dir)
				[keep_dir, turn_dir] = move_instruction(direction, cur_dir)
		'''

		#calculate the distance between current position and the next check point
		#cur_distance = math.sqrt(math.pow((cur_pos_x - new_graph.V[path[index]].x),2) + math.pow((cur_pos_y - new_graph.V[path[index]].y),2))
		
		cur_distance = math.sqrt(math.pow((new_graph.V[path[index-1]].x - new_graph.V[path[index]].x),2) + math.pow((new_graph.V[path[index-1]].y - new_graph.V[path[index]].y),2))
		#calibrate the current direction to the correct direction
		#cur_dir = dir_calibrate(keep_dir, cur_dir)

		#output and get the new position
		print('the curent direction  is: ' + str(round(cur_dir,2)))
		system("echo the current direction is " + str(round(cur_dir,2)) + " | festival --tts")
		#espeak.synth('direction is ' + str(round(cur_dir,2)))
		print('the rest distance is: ' + str(round(cur_distance,2)))
		system("echo the rest distance is " + str(round(cur_distance,2)) + " | festival --tts")
		#espeak.synth('distance is ' + str(round(cur_distance,2)) + 'centimeters')
		
		if ins_flag == 1 or first_flag == 0:
			#send_ins(turn_dir, cur_distance, index, length)
			#index = index + 1
			print('index is: '+ str(index))
			send_ins(new_graph, turn_dir, cur_distance, index, length, path)
			#ok_cnt = ok_cnt + 1
			first_flag = 1
			ins_flag = 0

		#send the instruction to Arduno
		[cur_pos_x, cur_pos_y, cur_dir, ins_flag] = cal_position(cur_pos_x, cur_pos_y, north_dir)  #"cur_dir" and "dis" are got from Arduino
		cur_dir = direction #00000000000
                
                
		#cur_pos_x = int(input('please input current X position:'))
		#cur_pos_y = int(input('please input current Y position:'))
		#cur_dir = int(input('please input current direction:')
                
		#[cur_pos_x, cur_pos_y, cur_dir, ins_flag] = cal_position(cur_pos_x, cur_pos_y, north_dir)  #"cur_dir" and "dis" are got from Arduino
		#ins_flag = 0
		
	#send_terminal()
	print('arrive at the connection !')
	system("echo arrive at the connection | festival --tts")
	#espeak.synth('arrive at the connection !')





	#run the dijkstra and find the path (a list of vertices' id
	# vertice 31 is the connection between 2 building
	path = dijkstra(cross_graph, int(1), int(end_p)) 
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

	#start the routing on the second map
	index = 0
	ins_flag = 0
	cur_pos_x = 0
	cur_pos_y = 0
	
	first_flag = 0
	send_req()
	cur_pos_x = cross_graph.V[path[0]].x
	cur_pos_y = cross_graph.V[path[0]].y
	[cur_pos_x, cur_pos_y, cur_dir, ins_flag] = cal_position(cur_pos_x, cur_pos_y, north_dir)  #"cur_dir" and "dis" are got from Arduino
	ins_flag = 1
	ok_cnt = 1
	keep_dir = cur_dir
	first_dir_flag = 1
	while (check_point_area(cur_pos_x, cur_pos_y, cross_graph.V[path[length-1]]) == 0 and ok_cnt < length):

		tmp_point = Vertex('temp', 0, cur_pos_x, cur_pos_y)
		index = index + 1
		ok_cnt = ok_cnt + 1
		if first_dir_flag == 1:
			direction = tmp_point.rel_angle(cross_graph.V[path[index]].x, cross_graph.V[path[index]].y, north_dir)
			first_dir_flag = 0
		elif first_dir_flag == 0:
			direction = cross_graph.V[path[index-1]].rel_angle(cross_graph.V[path[index]].x, cross_graph.V[path[index]].y, north_dir)
		#direction = tmp_point.rel_angle(cross_graph.V[path[index]].x, cross_graph.V[path[index]].y, north_dir)
		#get and output the instruction of current movement 
		[keep_dir, turn_dir] = move_instruction(direction, cur_dir)
		'''
		if (check_point_area(cur_pos_x, cur_pos_y, cross_graph.V[path[index]]) == 1):

			#get the correct direction pointing at the next check point
			index = index + 1
			ok_cnt = ok_cnt + 1
			direction = tmp_point.rel_angle(cross_graph.V[path[index]].x, cross_graph.V[path[index]].y, 
				north_dir)
			#get and output the instruction of current movement 
			[keep_dir, turn_dir] = move_instruction(direction, cur_dir)
		else:
			reset_start = check_wrong_route(cur_pos_x, cur_pos_y, cross_graph, index)
			if (reset_start != index and reset_start != -1):
				reset_start = reset_start + 1
				reset_flag = 1
				print('Wrong route ! Reset route now!')
				#espeak.synth('Wrong route ! ')
				if (reset_flag == 1):
					start_p = reset_start
				reset_flag = 0	
				path = dijkstra(cross_graph, int(start_p), int(end_p)) 
				length = len(path)
				#scale vertices' id to the index of the graph_matrix
				print('new path is :' + str(path))
				for i in range(0,length):
					path[i] = path[i] - 1

				#get the correct direction pointing at the next check point
				index = 0
				cur_pos_x = cross_graph.V[path[0]].x
				cur_pos_y = cross_graph.V[path[0]].y
				index = index + 1
				ok_cnt = ok_cnt + 1
				tmp_point = Vertex('temp', 0, cur_pos_x, cur_pos_y)
				direction = tmp_point.rel_angle(cross_graph.V[path[index]].x, cross_graph.V[path[index]].y, 
					north_dir)
				#get and output the instruction of current movement 
				[keep_dir, turn_dir] = move_instruction(direction, cur_dir)
			elif (reset_start == -1):
				direction = tmp_point.rel_angle(cross_graph.V[path[index]].x, cross_graph.V[path[index]].y, 
					north_dir)
				[keep_dir, turn_dir] = move_instruction(direction, cur_dir)
		'''

		#calculate the distance between current position and the next check point
		#cur_distance = math.sqrt(math.pow((cur_pos_x - cross_graph.V[path[index]].x),2) + math.pow((cur_pos_y - cross_graph.V[path[index]].y),2))
		
		cur_distance = math.sqrt(math.pow((cross_graph.V[path[index-1]].x - cross_graph.V[path[index]].x),2) + math.pow((cross_graph.V[path[index-1]].y - cross_graph.V[path[index]].y),2))
		#calibrate the current direction to the correct direction
		#cur_dir = dir_calibrate(keep_dir, cur_dir)

		#output and get the new position
		print('the curent direction  is: ' + str(round(cur_dir,2)))
		system("echo the current direction is " + str(round(cur_dir,2)) + " | festival --tts")
		#espeak.synth('direction is ' + str(round(cur_dir,2)))
		print('the rest distance is: ' + str(round(cur_distance,2)))
		system("echo the rest distance is " + str(round(cur_distance,2)) + " | festival --tts")
		#espeak.synth('distance is ' + str(round(cur_distance,2)) + 'centimeters')
		
		if ins_flag == 1 or first_flag == 0:
			#send_ins(turn_dir, cur_distance, index, length)
			send_ins(cross_graph, turn_dir, cur_distance, index, length, path)
			
			#index = index + 1
			print('index is: '+ str(index))
			#ok_cnt = ok_cnt + 1
			first_flag = 1
			ins_flag = 0

		#send the instruction to Arduno
		[cur_pos_x, cur_pos_y, cur_dir, ins_flag] = cal_position(cur_pos_x, cur_pos_y, north_dir)  #"cur_dir" and "dis" are got from Arduino
		cur_dir = direction #00000000000
                
                
		#cur_pos_x = int(input('please input current X position:'))
		#cur_pos_y = int(input('please input current Y position:'))
		#cur_dir = int(input('please input current direction:')
                
		#[cur_pos_x, cur_pos_y, cur_dir, ins_flag] = cal_position(cur_pos_x, cur_pos_y, north_dir)  #"cur_dir" and "dis" are got from Arduino
		#ins_flag = 0
		
	send_terminal()
	print('arrive the at the destination !')
	system("echo you have reached the destination | festival --tts")
	#espeak.synth('arrive at the destination !')


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
	elif (target.distance(cur_pos_x,cur_pos_y) < 95):
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

	return [direction, final_direction]
    
    
def move_instruction(direction, cur_dir):
	final_direction = 0
	if direction < 0:
		direction = direction + 360	#rescale the relative direction to 0-360

	final_direction = direction - cur_dir

	if final_direction > 0:
            if final_direction <= 180:
                print('turn right for ' + str(round(final_direction,0)) + ' degrees !')
                #espeak.synth('turn right ' + str(round(final_direction,0)) + ' degrees !')
                system("echo turn right for " + str(round(final_direction,0)) + " degrees | festival --tts")
            elif final_direction > 180 and final_direction <=360:
                final_direction = final_direction - 360
                print('turn left for ' + str(round(final_direction,0)) + ' degrees !')
                #espeak.synth('turn left ' + str(round(final_direction,0)) + ' degrees !')
                system("echo turn left for " + str(round(final_direction,0)) + " degrees | festival --tts")
	elif final_direction < 0:
            if (-final_direction) <= 180:
                print('turn left ' + str(round(final_direction,0)) + ' degrees !')
                #espeak.synth('turn left for ' + str(round(final_direction,0)) + ' degrees !')
                system("echo turn left for " + str(round(final_direction,0)) + " degrees | festival --tts")
            elif (-final_direction)> 180 and (-final_direction)<=360 :
                final_direction = final_direction + 360
                print('turn right ' + str(round(final_direction,0)) + ' degrees !')
                #espeak.synth('turn right ' + str(round(final_direction,0)) + ' degrees !')
                system("echo turn right for " + str(round(final_direction,0)) + " degrees | festival --tts")
                
	else:
		print('please go straight forward !')
		#espeak.synth('go straight!')
		system("echo go straight please| festival --tts")

	return [direction, final_direction]

'''
This function will calibrate the current direction to the correct direction to next check point (keep_dir)
final_direction : the degrees that the current direction needs to change
'''
def dir_calibrate(keep_dir, cur_dir):
	if keep_dir != cur_dir:
		#print('-----calibrate direction-----')
		final_direction = keep_dir - cur_dir
		'''
		if (final_direction > 0):
                    #print('turn the direction right for ' + str(round(final_direction,2)) + ' degrees !')
                    ##espeak.synth('turn the direction right for ' + str(final_direction) + ' degrees !')
                elif (final_direction < 0):
                    #print('turn the direction left for ' + str(-round(final_direction,2)) + ' degrees !')
                    ##espeak.synth('turn the direction left for ' + str(-final_direction) + ' degrees !')
		'''
		print('-----calibrate direction-----')
		##espeak.synth('the direction has been calibrated')
		return keep_dir
	return keep_dir