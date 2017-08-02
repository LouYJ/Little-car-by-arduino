def parse_ins(in_str):
    # op(2) + dis(4) + dir(2)
    op = in_str[0:2]
    dis = in_str[2:6]
    direction = in_str[6:10]
    print(op + " " + dis + " " + direction)
    if op == '31':
        ret_dis = int(dis)
        ret_dir = int(direction)
        print("ret_dis/ret_dir = " + str(ret_dis) + "/" + str(ret_dir))
        
        
    
    
    #str(bytes([0x11]),'utf-8')
if __name__ == '__main__':
    in_str = "3122224444"
    parse_ins(in_str)
