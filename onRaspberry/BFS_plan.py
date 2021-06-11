def BFS(graph,start_porint) :
    queue=[]
    queue.append(start_porint)
    seen=set()
    seen.add(start_porint)
    parent={start_porint:None}
    while queue :
        vertex=queue.pop(0)
        nodes=graph[vertex]
        for w in nodes:
            if w not in seen:
                queue.append(w)
                seen.add(w)
                parent[w]=vertex
        # print(vertex)
    return parent

def calculate_path(finishing_point,parent):
    path_point=finishing_point
    path=[]
    while path_point !=None :
        path.insert(0,path_point)
        path_point=parent[path_point]

    return path


# 顶层封装函数，使用BFS规划路径
def calculate_BFS(map, start_point, finishing_point):
    parent = BFS(map,start_point)
    # print(parent["C0"])
    path = calculate_path(finishing_point,parent)
    return path

# 存储有向图的邻接链表
# X:[left,down,up,right]
map={
    'A0':['B0'],
    'A1':['B1'],
    'A2':['B2','A3'],
    'A3':['A2','B3','A4'],
    'A4':['A3','B4','A5'],
    'A5':['A4','B5','A6'],
    'A6':['A5','B6','A7'],
    'A7':['A6','B7','A8'],
    'A8':['A7','B8','A9'],
    'A9':['A8','B9'],
    'B0':['A0','C0','B1'],
    'B1':['B0','A1','C1','B2'],
    'B2':['B1','C2','B3'], # B2 -> A2 去掉了，防止起点区的红色导致的误判
    'B3':['B2','C3','B4'],
    'B4':['B3','A4','C4','B5'],
    'B5':['B4','A5','C5','B6'],
    'B6':['B5','A6','C6','B7'],
    'B7':['B6','A7','C7','B8'],
    'B8':['B7','A8','C8','B9'],
    'B9':['B8','A9','C9'],
    'C0':['B0','D0','C1'],
    'C1':['C0','B1','D1','C2'],
    'C2':['C1','B2','D2','C3'],
    'C3':['C2','B3','C4'],
    'C4':['C3','B4','C5'],
    'C5':['C4','B5','C6'],
    'C6':['C5','B6','C7'],
    'C7':['C6','B7','D7','C8'],
    'C8':['C7','B8','D8','C9'],
    'C9':['C8','B9','D9'],
    'D0':['C0','E0','D1'],
    'D1':['D0','C1','E1','D2'],
    'D2':['D1','C2','E2'],
    'D7':['C7','E7','D8'],
    'D8':['D7','C8','E8','D9'],
    'D9':['D8','C9','E9'],
    'E0':['D0','F0','E1'],
    'E1':['E0','D1','F1','E2'],
    'E2':['E1','D2','F2'],
    'E7':['D7','F7','E8'],
    'E8':['E7','D8','F8','E9'],
    'E9':['E8','D9','F9'],
    'F0':['E0','G0','F1'],
    'F1':['F0','E1','G1','F2'],
    'F2':['F1','E2','G2'],
    'F7':['E7','G7','F8'],
    'F8':['F7','E8','G8','F9'],
    'F9':['F8','E9','G9'],
    'G0':['F0','H0','G1'],
    'G1':['G0','F1','H1','G2'],
    'G2':['G1','F2','H2'],
    'G7':['F7','H7','G8'],
    'G8':['G7','F8','H8','G9'],
    'G9':['G8','F9','H9'],
    'H0':['G0','I0','H1'],
    'H1':['H0','G1','I1','H2'],
    'H2':['H1','G2','I2','H3'],
    'H3':['H2','I3','H4'],
    'H4':['H3','H4','H5'],
    'H5':['H4','I5','H6'],
    'H6':['H5','I6','H7'],
    'H7':['H6','G7','I7','H8'],
    'H8':['H7','G8','I8','H9'],
    'H9':['H8','G9','I9'],
    'I0':['H0','J0','I1'],
    'I1':['I0','H1','J1','I2'],
    'I2':['I1','H2','J2','I3'],
    'I3':['I2','H3','J3','I4'],
    'I4':['I3','H4','J4','I5'],
    'I5':['I4','H5','J5','I6'],
    'I6':['I5','H6','J6','I7'],
    'I7':['I6','H7','J7','I8'],
    'I8':['I7','H8','J8','I9'],
    'I9':['I8','H9','J9'],
    'J0':['I0','J1'],
    'J1':['J0','I1','J2'],
    'J2':['J1','I2','J3'],
    'J3':['J2','I3','J4'],
    'J4':['J3','I4','J5'],
    'J5':['J4','I5','J6'],
    'J6':['J5','I6','J7'],
    'J7':['J6','I7','J8'],
    'J8':['J7','I8','J9'],
    'J9':['J8','I9'],
    
}



if __name__ == '__main__':
    # parent=BFS(map,'A2')
    # path=calculate_path('C0',parent)
    path = calculate_BFS(map, 'A2', 'C0')
    print(path)
    # parent = {"a0":1}
    # parent["c0"]

# print(calculate_BFS(map,'C0', 'I8'))