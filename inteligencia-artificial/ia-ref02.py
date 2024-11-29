class Node:
    def __init__(self, position, cost):
        self.position = position
        self.cost = cost

def uniform_cost_search(map, start, goal):
    frontier = [Node(start, 0)]
    visited = set()

    while frontier:
        current = frontier.pop(0)
        visited.add(current.position)

        if current.position == goal:
            return current.cost, visited, frontier

        for neighbor in map.get_neighbors(current.position):
            if neighbor not in visited:
                frontier.append(Node(neighbor, current.cost + 1))

def best_first_search(map, start, goal):
    frontier = [Node(start, 0)]
    visited = set()

    while frontier:
        current = frontier.pop(0)
        visited.add(current.position)

        if current.position == goal:
            return current.cost, visited, frontier

        neighbors = map.get_neighbors(current.position)
        for neighbor in neighbors:
            if neighbor not in visited:
                frontier.append(Node(neighbor, current.cost + 1))
        frontier.sort(key=lambda x: x.cost)

def a_star_search(map, start, goal):
    frontier = [Node(start, 0)]
    visited = set()

    while frontier:
        current = frontier.pop(0)
        visited.add(current.position)

        if current.position == goal:
            return current.cost, visited, frontier

        neighbors = map.get_neighbors(current.position)
        for neighbor in neighbors:
            if neighbor not in visited:
                frontier.append(Node(neighbor, current.cost + 1))
        frontier.sort(key=lambda x: x.cost + map.get_heuristic(x.position, goal))


###########################################################


import heapq
import numpy as np
import matplotlib.pyplot as plt

# Definindo constantes para movimento
MOVES = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # direita, esquerda, baixo, cima
MOVE_COST = 1

# Classe para representar o nó no espaço de busca
class Node:
    def __init__(self, position, cost):
        self.position = position
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

# Função para checar se a posição está dentro do mapa
def is_valid_move(grid_shape, position):
    return 0 <= position[0] < grid_shape[0] and 0 <= position[1] < grid_shape[1]

# Algoritmo de busca de custo uniforme
def uniform_cost_search(start, goal, grid):
    visited = set()
    frontier = []
    heapq.heappush(frontier, start)
    came_from = {}

    while frontier:
        current = heapq.heappop(frontier)
        visited.add(current.position)

        if current.position == goal.position:
            return came_from

        for move in MOVES:
            next_pos = (current.position[0] + move[0], current.position[1] + move[1])
            if is_valid_move(grid.shape, next_pos) and grid[next_pos] != 1 and next_pos not in visited:
                new_cost = current.cost + MOVE_COST
                neighbor = Node(next_pos, new_cost)
                heapq.heappush(frontier, neighbor)
                came_from[next_pos] = current.position

    return None

# Algoritmo de busca de melhor primeiro
def best_first_search(start, goal, grid):
    visited = set()
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}

    while frontier:
        _, current = heapq.heappop(frontier)
        visited.add(current.position)

        if current.position == goal.position:
            return came_from

        for move in MOVES:
            next_pos = (current.position[0] + move[0], current.position[1] + move[1])
            if is_valid_move(grid.shape, next_pos) and grid[next_pos] != 1 and next_pos not in visited:
                priority = heuristic(next_pos, goal.position)  # Prioridade baseada na heurística
                neighbor = Node(next_pos, priority)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[next_pos] = current.position

    return None

# Algoritmo A*
def a_star_search(start, goal, grid):
    visited = set()
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {start.position: 0}

    while frontier:
        _, current = heapq.heappop(frontier)
        visited.add(current.position)

        if current.position == goal.position:
            return came_from

        for move in MOVES:
            next_pos = (current.position[0] + move[0], current.position[1] + move[1])
            new_cost = cost_so_far[current.position] + MOVE_COST
            if is_valid_move(grid.shape, next_pos) and grid[next_pos] != 1 and (next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]):
                cost_so_far[next_pos] = new_cost
                priority = new_cost + heuristic(next_pos, goal.position)  # f(n) = g(n) + h(n)
                neighbor = Node(next_pos, priority)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[next_pos] = current.position

    return None

# Função de heurística (distância de Manhattan)
def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

# Função para desenhar o mapa
def draw_map(grid, start, goal, path):
    grid_with_path = grid.copy()
    for pos in path:
        grid_with_path[pos] = 2  # marcando o caminho percorrido
    grid_with_path[start.position] = 3  # marcando a posição inicial
    grid_with_path[goal.position] = 4  # marcando a posição do objetivo

    plt.imshow(grid_with_path, cmap='viridis', interpolation='nearest')
    plt.title('Mapa com o caminho encontrado')
    plt.show()

# Função principal
def main():
    # Definindo o mapa
    grid = np.array([
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ])

    start = Node((0, 0), 0)
    goal = Node((4, 4), 0)

    # Busca de custo uniforme
    print("Busca de custo uniforme:")
    uniform_came_from = uniform_cost_search(start, goal, grid)
    path = []
    current = goal.position
    while current != start.position:
        path.append(current)
        current = uniform_came_from[current]
    path.append(start.position)
    path.reverse()
    print("Caminho:", path)
    draw_map(grid, start, goal, path)

    # Busca de melhor primeiro
    print("Busca de melhor primeiro:")
    best_first_came_from = best_first_search(start, goal, grid)
    path = []
    current = goal.position
    while current != start.position:
        path.append(current)
        current = best_first_came_from[current]
    path.append(start.position)
    path.reverse()
    print("Caminho:", path)
    draw_map(grid, start, goal, path)

    # Busca A*
    print("Busca A*:")
    a_star_came_from = a_star_search(start, goal, grid)
    path = []
    current = goal.position
    while current != start.position:
        path.append(current)
        current = a_star_came_from[current]
    path.append(start.position)
    path.reverse()
    print("Caminho:", path)
    draw_map(grid, start, goal, path)

if __name__ == "__main__":
    main()