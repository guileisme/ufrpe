import numpy as np
import cv2

# Definindo os valores dos nós no mapa
FREE = 0
OBSTACLE = 1
START = 2
GOAL = 3
FRONTIER = 4
VISITED = 5
PATH = 6

# Definindo as cores para visualização
colors = {
    FREE: (255, 255, 255),     # Branco
    OBSTACLE: (0, 0, 0),        # Preto
    START: (0, 255, 0),         # Verde
    GOAL: (0, 0, 255),          # Azul
    FRONTIER: (255, 255, 0),    # Amarelo
    VISITED: (128, 128, 128),   # Cinza
    PATH: (255, 0, 0)           # Vermelho
}

# Função para visualizar o mapa
def visualize_map(map):
    map_rgb = np.zeros((map.shape[0], map.shape[1], 3), dtype=np.uint8)
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            map_rgb[i, j] = colors[map[i, j]]
    resolucao = cv2.resize(map_rgb, dsize=(300, 300), interpolation=cv2.INTER_NEAREST_EXACT)
    cv2.imshow('Map', resolucao)
    cv2.waitKey(0)

# Função para encontrar vizinhos válidos de um nó
def find_neighbors(map, node):
    neighbors = []
    rows, cols = map.shape
    row, col = node
    # Movimentos possíveis: acima, abaixo, esquerda, direita
    deltas = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for dr, dc in deltas:
        r, c = row + dr, col + dc
        if 0 <= r < rows and 0 <= c < cols and map[r, c] != OBSTACLE:
            neighbors.append((r, c))
    return neighbors

# Implementação do algoritmo de busca de custo uniforme
def uniform_cost_search(map, start, goal):
    rows, cols = map.shape
    visited = np.zeros((rows, cols), dtype=bool)
    frontier = [(0, start)]  # (cost, node)
    while frontier:
        cost, node = min(frontier)
        frontier.remove((cost, node))
        row, col = node
        if node == goal:
            break
        visited[row, col] = True
        map[row, col] = VISITED
        visualize_map(map)
        for neighbor in find_neighbors(map, node):
            if not visited[neighbor]:
                frontier.append((cost + 1, neighbor))
                map[neighbor] = FRONTIER
    path = [node]
    while node != start:
        for neighbor in find_neighbors(map, node):
            if map[neighbor] == VISITED and (neighbor[0] - node[0]) ** 2 + (neighbor[1] - node[1]) ** 2 == 1:
                path.append(neighbor)
                node = neighbor
                break
    for node in path:
        map[node] = PATH
        visualize_map(map)

# Implementação do algoritmo de busca melhor primeiro
def best_first_search(map, start, goal):
    rows, cols = map.shape
    visited = np.zeros((rows, cols), dtype=bool)
    frontier = [(heuristic(start, goal), start)]  # (heuristic, node)
    while frontier:
        _, node = min(frontier)
        frontier.remove((_, node))
        row, col = node
        if node == goal:
            break
        visited[row, col] = True
        map[row, col] = VISITED
        visualize_map(map)
        for neighbor in find_neighbors(map, node):
            if not visited[neighbor]:
                frontier.append((heuristic(neighbor, goal), neighbor))
                map[neighbor] = FRONTIER
    path = [node]
    while node != start:
        for neighbor in find_neighbors(map, node):
            if map[neighbor] == VISITED and (neighbor[0] - node[0]) ** 2 + (neighbor[1] - node[1]) ** 2 == 1:
                path.append(neighbor)
                node = neighbor
                break
    for node in path:
        map[node] = PATH
        visualize_map(map)

# Implementação do algoritmo A*
def A_star_search(map, start, goal):
    rows, cols = map.shape
    visited = np.zeros((rows, cols), dtype=bool)
    frontier = [(heuristic(start, goal), 0, start)]  # (f_cost, g_cost, node)
    while frontier:
        _, g_cost, node = min(frontier)
        frontier.remove((_, g_cost, node))
        row, col = node
        if node == goal:
            break
        visited[row, col] = True
        map[row, col] = VISITED
        visualize_map(map)
        for neighbor in find_neighbors(map, node):
            if not visited[neighbor]:
                new_g_cost = g_cost + 1
                new_f_cost = new_g_cost + heuristic(neighbor, goal)
                frontier.append((new_f_cost, new_g_cost, neighbor))
                map[neighbor] = FRONTIER
    path = [node]
    while node != start:
        for neighbor in find_neighbors(map, node):
            if map[neighbor] == VISITED and (neighbor[0] - node[0]) ** 2 + (neighbor[1] - node[1]) ** 2 == 1:
                path.append(neighbor)
                node = neighbor
                break
    for node in path:
        map[node] = PATH
        visualize_map(map)

# Função de heurística (distância de Manhattan)
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Função principal
def main():
    # Definição do mapa de exemplo (0 representa espaço livre, 1 representa obstáculo)
    map = np.array([
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 0, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ])

    # Definição da posição inicial e do objetivo
    start = (0, 0)
    goal = (4, 3)

    # Visualização do mapa inicial
    visualize_map(map)

    # Chamada dos algoritmos de busca
    # uniform_cost_search(map.copy(), start, goal)
    # best_first_search(map.copy(), start, goal)
    A_star_search(map.copy(), start, goal)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()