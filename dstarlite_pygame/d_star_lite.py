import heapq
from utils import stateNameToCoords
from math import sqrt

def heuristic(nodeA, nodeB):
    # nodeA와 nodeB 사이의 대각선 거리를 계산하여 반환합니다.
    dx = abs(int(nodeA.split('x')[1][0]) - int(nodeB.split('x')[1][0]))  # nodeA와 nodeB의 x 좌표 차이의 절댓값
    dy = abs(int(nodeA.split('y')[1][0]) - int(nodeB.split('y')[1][0]))  # nodeA와 nodeB의 y 좌표 차이의 절댓값
    return ((sqrt(2.0)-1.0)*min(dx, dy) + max(dx, dy))  # 대각선으로 이동 가능한 최소 거리와 직선 거리를 합친 휴리스틱 값 반환

def topKey(heap):
    # 우선순위 큐(heap)에서 최소 키 값을 반환합니다.
    heap.sort()
    if len(heap) > 0:
        return heap[0][:2]
    else:
        return (float('inf'), float('inf'))

def calculateKey(graph, s, s_start, k_m):
    # 주어진 노드 s에 대한 키 값을 계산하여 반환합니다.
    comp = min(graph.graph[s].g, graph.graph[s].rhs)  # g 값과 rhs 값 중 더 작은 값을 comp로 설정
    key1 = comp + heuristic(s_start, s) + k_m  # 키 값 계산
    return (key1, comp)

def removeFromHeap(graph, heap, s):
    # 주어진 노드 s를 우선순위 큐(heap)에서 제거합니다.
    if (graph.graph[s].inHeap):
        id_in_heap = [item for item in heap if s in item]
        if id_in_heap != []:
            if len(id_in_heap) != 1:
                raise ValueError('more than one ' + id + ' in the queue!')
            heap.remove(id_in_heap[0])
    graph.graph[s].inHeap = False

def popHeap(graph, heap):
    # 우선순위 큐(heap)에서 최소 키 값을 가진 노드를 제거하고 반환합니다.
    poppedNode = heapq.heappop(heap)[2]
    graph.graph[poppedNode].inHeap = False
    return poppedNode

def insert(graph, heap, s, key):
    # 주어진 노드 s를 우선순위 큐(heap)에 삽입합니다.
    heapq.heappush(heap, key + (s,))
    graph.graph[s].inHeap = True

def updateVertex(graph, heap, u, s_start, k_m):
    # 주어진 노드 u의 g 값과 rhs 값을 업데이트하고, 필요한 경우 우선순위 큐(heap)에 삽입합니다.
    s_goal = graph.goal
    if u != s_goal:
        min_rhs = float('inf')
        for n in graph.graph[u].successors:
            min_rhs = min(min_rhs, graph.graph[n].g + graph.graph[u].successors[n])
        graph.graph[u].rhs = min_rhs
    removeFromHeap(graph, heap, u)
    if graph.graph[u].rhs != graph.graph[u].g:
        insert(graph, heap, u, calculateKey(graph, u, s_start, k_m))

def computeShortestPath(graph, heap, s_start, k_m):
    # D* Lite 알고리즘을 사용하여 최단 경로를 계산합니다.
    while (graph.graph[s_start].rhs != graph.graph[s_start].g) or (topKey(heap) < calculateKey(graph, s_start, s_start, k_m)):
        k_old = topKey(heap)
        u = popHeap(graph, heap)
        if k_old < calculateKey(graph, u, s_start, k_m):
            insert(graph, heap, u, calculateKey(graph, u, s_start, k_m))
        elif graph.graph[u].g > graph.graph[u].rhs:
            graph.graph[u].g = graph.graph[u].rhs
            for n in graph.graph[u].neighbors:
                neighbor_coords = stateNameToCoords(n)
                if (graph.cells[neighbor_coords[1]][neighbor_coords[0]] != -1):  # 장애물이 아닌 경우에만 업데이트
                    updateVertex(graph, heap, n, s_start, k_m)
        else:
            graph.graph[u].g = float('inf')
            updateVertex(graph, heap, u, s_start, k_m)
            for n in graph.graph[u].neighbors:
                neighbor_coords = stateNameToCoords(n)
                if (graph.cells[neighbor_coords[1]][neighbor_coords[0]] != -1):  # 장애물이 아닌 경우에만 업데이트
                    updateVertex(graph, heap, n, s_start, k_m)

def nextInShortestPath(graph, s_current):
    # 현재 노드에서 다음 노드로 이동하기 위한 함수입니다.
    min_rhs = float('inf')
    s_next = None
    if graph.graph[s_current].rhs == float('inf'):
        print('There is no known path to the goal')
    else:
        for i in graph.graph[s_current].successors:
            child_cost = graph.graph[i].g + graph.graph[s_current].successors[i]
            if (child_cost) < min_rhs:
                min_rhs = child_cost
                s_next = i
        if s_next:
            return s_next
        else:
            raise ValueError('could not find child for transition!')

def scanForObstacles(graph, heap, s_current, scan_range, k_m):
    # 현재 위치에서 주어진 범위 내의 장애물을 스캔하고, 필요한 경우 그래프를 업데이트합니다.
    states_to_update = {}
    range_checked = 0
    if scan_range >= 1:
        for neighbor in graph.graph[s_current].successors:
            neighbor_coords = stateNameToCoords(neighbor)
            states_to_update[neighbor] = graph.cells[neighbor_coords[1]][neighbor_coords[0]]
        range_checked = 1

    while range_checked < scan_range:
        new_set = {}
        for state in states_to_update:
            new_set[state] = states_to_update[state]
            for neighbor in graph.graph[state].successors:
                if neighbor not in new_set:
                    neighbor_coords = stateNameToCoords(neighbor)
                    new_set[neighbor] = graph.cells[neighbor_coords[1]][neighbor_coords[0]]
        range_checked += 1
        states_to_update = new_set

    new_obstacle = False
    for state in states_to_update:
        if states_to_update[state] < 0:  # 장애물이 있는 경우
            for neighbor in graph.graph[state].successors:
                if(graph.graph[state].successors[neighbor] != float('inf')):  # 이전에 장애물이 없었던 곳에 새로운 장애물이 발견된 경우
                    neighbor_coords = stateNameToCoords(state)
                    graph.cells[neighbor_coords[1]][neighbor_coords[0]] = -2  # 새로운 장애물로 표시
                    graph.graph[neighbor].successors[state] = float('inf')
                    graph.graph[state].successors[neighbor] = float('inf')
                    updateVertex(graph, heap, state, s_current, k_m)
                    new_obstacle = True
    return new_obstacle

def moveAndRescan(graph, heap, s_current, scan_range, k_m):
    # 현재 위치에서 다음 위치로 이동하고, 이동 후 주변을 스캔합니다.
    if(s_current == graph.goal):
        return 'goal', k_m
    else:
        s_last = s_current
        s_new = nextInShortestPath(graph, s_current)
        new_coords = stateNameToCoords(s_new)

        if(graph.cells[new_coords[1]][new_coords[0]] == -1):  # 새로운 장애물을 만난 경우
            s_new = s_current  # 현재 위치에 머물며 스캔 및 재계획을 수행

        results = scanForObstacles(graph, heap, s_new, scan_range, k_m)
        k_m += heuristic(s_last, s_new)  # k_m 값을 업데이트
        computeShortestPath(graph, heap, s_current, k_m)  # 최단 경로 재계산

        return s_new, k_m

def initDStarLite(graph, heap, s_start, s_goal, k_m):
    # D* Lite 알고리즘을 초기화합니다.
    graph.graph[s_goal].rhs = 0
    insert(graph, heap, s_goal, calculateKey(graph, s_goal, s_start, k_m))
    computeShortestPath(graph, heap, s_start, k_m)

    return (graph, heap, k_m)
