def dijkstra(G, A, B):
    inf = 99999
    fixed = []
    dist = []
    pre = []

    n = len(G.V)

    for i in range(1, n+2):
        dist.append(inf)
        pre.append(-1)

    dist[A] = 0
    pre[A] = A
    curNode = -1

    #print "n is ", n
    while (len(fixed) != n):
        min_dis = inf
        tmp = len(fixed)
        for i in range(1, n+1):
            if (i not in fixed) and (dist[i] < min_dis) :
                curNode = i
                #print i
                min_dis = dist[i]
                #print min_dis

        fixed.append(curNode)
        for i in range(1, n+1):
            #print "-------------------------"
            if (G.E[curNode-1][i-1] != -1) and (i not in fixed):
                if (dist[i] > dist[curNode] + G.E[curNode-1][i-1]):
                    dist[i] = dist[curNode] + G.E[curNode-1][i-1]
                    pre[i] = curNode
    '''
    print "The distance to B is: ", dist[B]
    print "The path from A to B is: "
    '''
    tmp_path = []
    path = []
    curNode = B

    tmp_path.append(curNode)
    while(pre[curNode] != curNode):
        tmp_path.append(pre[curNode])
        curNode = pre[curNode]

    n = len(tmp_path)

    while (n>0):
        path.append(tmp_path[n-1])
        n = n - 1

    return path


if __name__ == '__main__':

    V = [1, 2, 3, 4, 5, 6, 7]
    G = [[-1, 7, 9, -1, 3, -1, -1],
        [-1, -1, -1, 5, -1, -1, -1],
        [-1, -1, -1, 7, -1, -1, -1],
        [-1, -1, -1, -1, -1, 3, 8],
        [-1, -1, -1, 4, -1, 8, -1],
        [-1, -1, -1, -1, -1, -1, 4],
        [-1, 2, -1, -1, -1, -1, -1]]

    A = 1
    B = 7

    dijkstra(V, G, A, B)

