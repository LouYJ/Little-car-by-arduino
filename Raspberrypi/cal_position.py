import math
import serial
def cal_position(pre_x, pre_y, north_dir):
    #"rel_dir" and "dis" are got from Arduino
    
    ins_flag = 0
    in_str = get_ins()  #get instruction from Arduino
    if in_str == 'error' :
        print("error happens in getting instruction from Arduino !")
        
    [rel_dir, dis, ins_flag] = parse_ins(in_str)  # ok
    
    ret_x = 0
    ret_y = 0
    
    abs_dir = (rel_dir + north_dir) % 360
    
    abs_dir_rad = abs_dir / 180.0 * 3.1415926

    if abs_dir <=90:
        ret_x = pre_x + dis * math.sin(abs_dir_rad)
    elif abs_dir > 90 and abs_dir <= 180:
        ret_x = pre_x + dis * math.sin(abs_dir_rad)
    elif abs_dir > 180 and abs_dir < 270:
        ret_x = pre_x + dis  * math.sin(abs_dir_rad)
    else:
        ret_x = pre_x + dis  * math.sin(abs_dir_rad)
    
    
    if abs_dir <= 90:
        ret_y = pre_y + dis * math.cos(abs_dir_rad)
    elif abs_dir > 90 and abs_dir <= 180:
        ret_y = pre_y + dis * math.cos(abs_dir_rad)
    elif abs_dir > 180 and abs_dir < 270:
        ret_y = pre_y + dis * math.cos(abs_dir_rad)
    else:
        ret_y = pre_y + dis * math.cos(abs_dir_rad)
    print('x and y are: ' + str(ret_x) + '/' + str(ret_y))
    print('pre_x and pre_y are: ' + str(pre_x) + '/' + str(pre_y))
    return [int(ret_x), int(ret_y), int(rel_dir), ins_flag]



def parse_ins(in_str):
    # op(2) + dis(4) + dir(2)
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    op = in_str[0:2]
    dis = in_str[2:6]
    direction = in_str[6:10]
    #print(op + " " + dis + " " + direction)
    if op == '31':
        print('op == 31')
        ret_dis = int(dis,16)
        ret_dir = int(direction,16)
        print("ret_dis/ret_dir = " + str(ret_dis) + "/" + str(ret_dir))
        return [ret_dir, ret_dis, 0]
    elif op == '32':
        print('op == 32')
        ret_dis = int(dis,16)
        ret_dir = int(direction,16)
        print("ret_dis/ret_dir = " + str(ret_dis) + "/" + str(ret_dir))
        return [ret_dir, ret_dis, 1]
    return [0,0]
        
#send the request for current data
def send_req():
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    op = 2*16+0     #0x20
    turn_dir = 0
    distance = 0
    option = 0
    '''
    op_b = op.to_bytes(1, byteorder='big')
    distance_b = distance.to_bytes(2, byteorder='big') 
    cur_dir_b = cur_dir.to_bytes(1, byteorder='big')
    option_b = option.to_bytes(1, byteorder='big')
    '''
    op_b = hex(op)[2::].zfill(2)
    distance_b = hex(distance)[2::].zfill(4)
    turn_dir_b = hex(turn_dir)[2::].zfill(2)
    option_b = hex(option)[2::].zfill(2)
    
    ins_str = str(op_b) + str(distance_b) + str(turn_dir_b) + str(option_b)
    ser.write(ins_str.encode('utf-8')) 

#get the str from Arduino
def get_ins():
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    extend = 0
    extend_b = extend.to_bytes(4, byteorder='big')
    try:
        while 1:
            ack = 17  #ack is 0x11
            ack_b = int(ack).to_bytes(1, byteorder = 'big')
            send_str = str(ack_b) + str(extend_b)
            response = ser.readline()
            ret_str = str(response, 'utf-8')
            #print("ret_str is : " + ret_str)
            if (len(ret_str) != 0):
                ser.write("1300000000".encode('utf-8'))  #ack_data     0x13
                return ret_str
                break        
    except KeyboardInterrupt:
        ser.close()
        return 'error'

#send the instruction to Arduino
def send_ins(new_graph, turn_dir, distance,index,length, path):
    op = 0
    turn_dir = int(turn_dir)
    distance = int(distance)
    option = 0
    turn_flag = 0
    
    if turn_dir > 0:    # should turn right
        op = 2*16 + 4   # 0x24
        turn_flag = 1

    elif turn_dir < 0:  # should turn left
        op = 2*16 + 3   # 0x23
        turn_dir = -turn_dir
        turn_flag = 1

    elif turn_dir == 0: # should go straight
        op = 2*16 + 1   # 0x21
        #print(str(new_graph.pid_banned) + 'vertex: ' )
        #print(path[index])
        if ((index < length-1) and (path[index] in new_graph.pid_banned) and (path[index+1] in new_graph.pid_banned)) :
            #print('ban pid in vertex:' + str(path[index]))
            turn_dir = 1  #不需要pid

        turn_flag = 0
        
    op_b = hex(op)[2::].zfill(2)
    distance_b = hex(distance)[2::].zfill(4)
    turn_dir_b = hex(turn_dir)[2::].zfill(2)
    option_b = hex(option)[2::].zfill(2)

    '''
    #楼蛋要的逗逼代码
    if (op == 2*16+1):
        send_p(index,length)
    '''

    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    ins_str = str(op_b) + str(distance_b) + str(turn_dir_b) + str(option_b)
    ser.write(ins_str.encode('utf-8')) 
    try:
        while 1:
            ack = 20  #ack is 0x14
            response = ser.readline()
            ret_str = str(response, 'utf-8')
            if (len(ret_str) != 0 and ret_str == "20"):
                print('ack 20 accepted')
                if turn_flag == 1:
                    '''
                    #楼蛋要的逗逼代码
                    send_p(index,length)
                    #loudan de shabi daima
                    '''
                    op = 2*16 + 1
                    op_b = hex(op)[2::].zfill(2) 
                    turn_dir = 0
                    print('now is '+ str(path[index]))
                    if ((index < length) and ((path[index]+1) in new_graph.pid_banned)):
                        #print('ban pid in vertex --turning:' + str(path[index]))
                        turn_dir = 1  #不需要pid
                        
                    turn_dir_b = hex(turn_dir)[2::].zfill(2)
                    ins_str = str(op_b) + str(distance_b) + str(turn_dir_b) + str(option_b)
                    ser.write(ins_str.encode('utf-8'))
                    
                    while 1:
                        ack = 20  #ack is 0x14
                        response = ser.readline()
                        ret_str = str(response, 'utf-8')
                        if (len(ret_str) != 0 and ret_str == "20"):
                            print('ack 20 accepted')
                            break
                break      
    except KeyboardInterrupt:
        ser.close()



def send_p(index, length):
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    if (index >= 2 and index <= length-1):
        index_b = hex(index)[2::].zfill(2)
        extend = 0
        extend_b = hex(extend)[2::].zfill(6)
        send_str = "26" + str(index_b) + str(extend)
        ser.write(send_str.encode('utf-8')) 
        print('send point_req : ' + str(index))

def send_terminal():
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    ser.write("2700000000".encode('utf-8')) 


if __name__ == '__main__':
    pass

    '''
    print(cal_position(0,0,30,30,100))
    print(cal_position(-100,60,-30,60,100))
    print(cal_position(-100,-100,30,30,100))
    print(cal_position(30,30,270,60,100))
    '''
    

