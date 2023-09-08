from curve import *
from vehicle import *
from road import *
from simulation import *
from window import *
from vehicle_generator import *
from traffic_signal import *
from parkingMap import parking_map

# Create simulation
sim = Simulation()

# 0 - Vacio
# 1 - Norte
# 2 - Este
# 3 - Sur
# 4 - Oeste
# 1000 - Estacionamiento hacia Norte
# 2000 - Estacionamiento hacia Este
# 3000 - Estacionamiento hacia Sur
# 4000 - Estacionamiento hacia Oeste

parking_width = 4
parking_depth = 8

roads = []
road_number = 0

# (0, 9) : 0
road_id = {}
# 15 : (1, 2)
parking_position = {}
# 1 : 34
parking_id = {}
for row in range(len(parking_map)):
    for col in range(len(parking_map[0])):
        road = parking_map[row][col]
        if road != 0:
            road_id[(row, col)] = road_number
            if road > 1000:
                parking_position[road_number] = (row, col)
                parking_id[road%1000] = road_number
            road_number += 1
     
def checkBoundaries(matrix, x, y):
    if x >= 0 and y >= 0 and x < len(matrix) and y < len(matrix[0]):
        if matrix[x][y] != 0:
            return True

def bfs(road_ids, matrix, starting_row, starting_col, end_id, parking_ids):
    queue = []
    starting_pos = (starting_row, starting_col)
    # (node, [path])
    queue.append((starting_pos, [road_ids[starting_pos]]))
    visited = set()
    end_parking_pos = parking_ids[end_id]
    end_x, end_y = end_parking_pos
    end_value = matrix[end_x][end_y]

    while queue:
        road, path = queue.pop(0)
        road_id = road_ids[road]
        x, y = road
        road_value = matrix[x][y]
        
        if road not in visited:
            visited.add(road)
            # Se encontro el final
            if road_id == end_id:
                return path
            else:
                # Norte
                if road_value != 3: # Checar que no estemos en una calle hacia el sur
                    if checkBoundaries(matrix, x-1, y):
                        new_road_id = road_ids[(x-1, y)]
                        new_road_value = matrix[x-1][y]
                        if new_road_value != 3:
                            # Revisar si es calle
                            if matrix[x-1][y] < 1000:
                                queue.append(((x-1, y), path + [new_road_id]))
                            # Es el estacionamiento final
                            elif (x-1, y) == end_parking_pos:
                                if end_value > 1000 and end_value < 2000:
                                    return path + [new_road_id]
                # Este
                if road_value != 4: # Checar que no estemos en una calle hacia el oeste
                    if checkBoundaries(matrix, x, y+1):
                        new_road_id = road_ids[(x, y+1)]
                        new_road_value = matrix[x][y+1]
                        if new_road_value != 4:
                            if matrix[x][y+1] < 1000:
                                queue.append(((x, y+1), path + [new_road_id]))
                            elif (x, y+1) == end_parking_pos:
                                if end_value > 2000 and end_value < 3000:
                                    return path + [new_road_id]
                # Sur
                if road_value != 1: # Checar que no estemos en una calle hacia el norte
                    if checkBoundaries(matrix, x+1, y):
                        new_road_id = road_ids[(x+1, y)]
                        new_road_value = matrix[x+1][y]
                        if new_road_value != 1:      
                            if matrix[x+1][y] < 1000:
                                queue.append(((x+1, y), path + [new_road_id]))
                            elif (x+1, y) == end_parking_pos:
                                if end_value > 3000 and end_value < 4000:
                                    return path + [new_road_id]
                # Oeste
                if road_value != 2: # Checar que no estemos en una calle hacia el este
                    if checkBoundaries(matrix, x, y-1):
                        new_road_id = road_ids[(x, y-1)]
                        new_road_value = matrix[x][y-1]
                        if new_road_value != 2:
                            if matrix[x][y-1] < 1000:
                                queue.append(((x, y-1), path + [new_road_id]))
                            elif (x, y-1) == end_parking_pos:
                                if end_value > 4000 and end_value < 5000:
                                    return path + [new_road_id]

for row_pos, row in enumerate(parking_map):
    for road_pos, road in enumerate(row):
        y = row_pos*parking_width
        x = road_pos*parking_width
        # Estacionamiento
        if road > 1000:
            # Apuntando al Norte
            if road < 2000:
                roads.append(((x, y), (x, y-parking_depth), 1))
            # Apuntando al Este
            elif road > 2000 and road < 3000:
                roads.append(((x-2, y-2), (x+parking_depth-2, y-2), 1))
            # Apuntando al Sur
            elif road > 3000 and road < 4000:
                roads.append(((x, y-parking_width), (x, y+parking_width), 1))
            # Apuntando al Oeste
            else:
                roads.append(((x+2, y-2), (x-parking_depth+2, y-2), 1))
        # Calle
        else:
            # Norte
            if road == 1:
                roads.append(((x,y), (x, y-parking_width)))
            # Este
            elif road == 2:
                roads.append(((x-2, y-2), (x+parking_width-2, y-2)))
            # Sur
            elif road == 3:
                roads.append(((x,y-parking_width), (x, y)))
            # Oeste
            elif road == 4:
                roads.append(((x+2, y-2), (x-parking_width+2, y-2)))

sim.create_roads(roads)

vehiculos = []

for i in range(618):
    # Se agrega la ruta de cada carro, dandole un peso de 1, para que todos tengan el mismo peso
    # Y partiendo de la posicion 17, 25
    vehiculos.append([1, {"path": bfs(road_id, parking_map, 79, 40, parking_id[i+1], parking_position)}])

# Creamos la simulacion con todos los vehiculos
# El vehicle rate nos ayuda a aumentar o disminuir la separacion de los carros
sim.create_gen({
    # Se recomienda 100, maximo 300
    'vehicle_rate': 200,
    'vehicles': vehiculos
})


# Start simulation
win = Window(sim)
win.offset = (0, 0)
win.run(steps_per_update=3)