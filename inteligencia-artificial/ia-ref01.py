import numpy as np
import matplotlib.pyplot as plt
import cv2
# from google.colab.patches import cv2_imshow
import time
# from IPython.display import clear_output

# definindo cores
start = (230, 20, 20)
goal = (20, 230, 20)
trail = (200, 100, 170)
selected = (200, 200, 200)
border = (230, 230, 230)
background = (255, 255, 255)
wall = (0, 0, 0)

# pintando um pixel
def paint(img, row, column, color) :
  img[row, column] = color

# mostrando a imagem
def show(img, width, height) :
  resized = cv2.resize(img, dsize=(width, height), interpolation=cv2.INTER_NEAREST_EXACT)
  # clear_output()
  cv2.imshow(resized)
def euclidean_distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def cost_function(map_aux, node_pos, goal_pos):
  x1 = node_pos[0]
  y1 = node_pos[1]
  x2 = goal_pos[0]
  y2 = goal_pos[0]
  distance_from_start = map_aux[x1, y1, 2]
  distance_to_goal = euclidean_distance(x1, y1, x2, y2)
  return distance_from_start

def pathfhind(map, start_pos, goal_pos):

  # clone do mapa com coordenadas x e y do nó pai e distâncias percorridas
  map_aux = np.zeros((50, 50, 3), dtype="uint8")

  # definindo que o pai do início é o próprio início e sua distância percorrida é 0
  map_aux[start_pos[0], start_pos[1], 0] = start_pos[0]
  map_aux[start_pos[0], start_pos[1], 1] = start_pos[1]
  map_aux[start_pos[0], start_pos[1], 2] = 0

  # incializando lista de nós da fronteira
  border_list = []

  # incializando lista de nós da marcados
  marked_list = []

  # inserindo o nó inicial na fronteira
  border_list.append(start_pos)

  # booleano para indicar que o caminho foi encontrado
  found = False

  # iterando na fronteira até encontrar o destino
  while len(border_list) > 0 and not found:

    time.sleep(0.1)
    show(map, 400, 400)

    # selecionando o nó da fronteira com menor custo
    selected_pos = border_list[0]
    selected_index = 0
    selected_cost = cost_function(map_aux, selected_pos, goal_pos)
    for index, item_pos in enumerate(border_list):
      item_cost = cost_function(map_aux, item_pos, goal_pos)
      if item_cost < selected_cost:
        selected_pos = item_pos
        selected_index = index
        selected_cost = item_cost

    # removendo o nó selecionado da fronteira
    border_list.pop(selected_index)

    # adicionando na lista de nós marcados
    marked_list.append(selected_pos)

    # pintando o nó selecionado
    if selected_pos != start_pos and selected_pos != goal_pos :
      paint(map, selected_pos[0], selected_pos[1], selected)

    # checando se o nó selecionado é o destino,
    # se for recuperamos o caminho (path)
    if selected_pos == goal_pos:
        path = []
        current_pos = selected_pos
        while current_pos != start_pos:
            # pintando a trilha do caminho
            if current_pos != start_pos and current_pos != goal_pos :
              paint(map, current_pos[0], current_pos[1], trail)

            # acessando os nós antecessores para completar o caminho
            path.append(current_pos)
            current_x = current_pos[0]
            current_y = current_pos[1]
            parent_x = map_aux[current_x, current_y, 0]
            parent_y = map_aux[current_x, current_y, 1]
            current_pos = (parent_x, parent_y)
            found = True

    # se o nó selecionado não é o objetivo, expandimos o nó
    else:
      for neighborhood_pos_offset in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # neighbor squares
          neighbor_x = selected_pos[0] + neighborhood_pos_offset[0]
          neighbor_y = selected_pos[1] + neighborhood_pos_offset[1]
          exp_pos = (neighbor_x, neighbor_y)
          map_width  = len(map)
          map_height = len(map[len(map)-1])

          # acessando somente vizinhos válidos
          if(neighbor_x >= 0 and neighbor_x < map_width and # x válido
            neighbor_y >= 0 and neighbor_y < map_height and # y válido
            tuple(map[neighbor_x, neighbor_y]) != wall  and # não é parede
            exp_pos not in marked_list                  and # não está marcado
            exp_pos not in border_list) :                   # ainda não está na fronteira

            # se este é um nó válido da expansão, então adicionamos ele na fronteira
            border_list.append(exp_pos)

            # pintando a fronteira
            if exp_pos != start_pos and exp_pos != goal_pos :
              paint(map, exp_pos[0], exp_pos[1], border)

            # e marcamos a posição do nó "mãe" no mapa auxiliar, e também a distância percorrida
            map_aux[exp_pos[0], exp_pos[1], 0] = selected_pos[0]
            map_aux[exp_pos[0], exp_pos[1], 1] = selected_pos[1]
            map_aux[exp_pos[0], exp_pos[1], 2] = map_aux[selected_pos[0], selected_pos[1], 2] + 1
# pintando o mapa
start_pos = (15, 15)
goal_pos = (22, 29)
paint(map, start_pos[0], start_pos[1], start)
paint(map, goal_pos[0], goal_pos[1], goal)
paint(map, 20, 10, wall)
paint(map, 20, 11, wall)
paint(map, 20, 12, wall)
paint(map, 20, 13, wall)
paint(map, 20, 14, wall)
paint(map, 20, 15, wall)
paint(map, 20, 16, wall)
paint(map, 20, 17, wall)
paint(map, 20, 18, wall)
paint(map, 20, 19, wall)
paint(map, 20, 20, wall)
paint(map, 20, 21, wall)
paint(map, 20, 22, wall)
paint(map, 20, 23, wall)
paint(map, 20, 24, wall)
paint(map, 20, 25, wall)
paint(map, 20, 26, wall)
paint(map, 20, 27, wall)
paint(map, 20, 28, wall)
paint(map, 20, 29, wall)
paint(map, 20, 30, wall)
paint(map, 19, 30, wall)
paint(map, 18, 30, wall)
paint(map, 17, 30, wall)
paint(map, 16, 30, wall)
paint(map, 15, 30, wall)

show(map, 400, 400)
pathfhind(map, start_pos, goal_pos)
show(map, 400, 400)

# https://stackoverflow.com/questions/62488398/display-animations-in-google-colab
# https://stackoverflow.com/questions/52162213/get-keyboard-events-on-jupyter-widgets