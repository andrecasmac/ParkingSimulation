from curve import *
from vehicle import *
from road import *
from simulation import *
from window import *
from vehicle_generator import *
from traffic_signal import *

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

parking_map = [[3,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	1,	0],
[3,	3,	3001,	3002,	3003,	3004,	3005,	3006,	3007,	3008,	3009,	3010,	3011,	3012,	3013,	3014,	3015,	3016,	3017,	3018,	3019,	3020,	3021,	1,	1,	0],
[3,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0],
[3,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0],
[3,	3,	1022,	1023,	1024,	1025,	1026,	1027,	1028,	1029,	1030,	1031,	1032,	1033,	1034,	1035,	1036,	1037,	1038,	1039,	1040,	1041,	1042,	1,	1,	0],
[2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	0],
[0,	0,	2,	3,	0,	0,	0,	3043,	3044,	3045,	3046,	3047,	3048,	3049,	3050,	3051,	3052,	3053,	3054,	3055,	3056,	3057,	3058,	1,	1,	0],
[0,	0,	0,	2,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0],
[0,	0,	0,	0,	2,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0],
[0,	0,	0,	0,	0,	2,	3,	1059,	1060,	1061,	1062,	1063,	1064,	1065,	1066,	1067,	1068,	1069,	1070,	1071,	1072,	1073,	1074,	1,	1,	0],
[0,	0,	0,	0,	0,	0,	3,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	1],
[0,	0,	0,	0,	0,	0,	2,	3,	0,	0,	0,	3075,	3076,	3077,	3078,	3079,	3080,	3081,	3082,	3083,	3084,	3085,	3086,	3087,	1,	1],
[0,	0,	0,	0,	0,	0,	0,	2,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1],
[0,	0,	0,	0,	0,	0,	0,	0,	2,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1],
[0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	3,	0,	0,	1088,	1089,	1090,	1091,	1092,	1093,	1094,	1095,	1096,	1097,	1098,	1,	1],
[0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1],
[0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	3099,	3100,	3101,	3102,	3103,	3104,	3105,	3106,	3107,	3108,	3109,	3110,	1],
[0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1]]


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

""" for i in range(110):
    sim.create_gen({
    'vehicle_rate': 1,
    'vehicles': [
        [i+1, {"path": bfs(road_id, parking_map, 17, 25, parking_id[i+1], parking_position)}]
        ]   
    }) """

for i in range(110):
    # Se agrega la ruta de cada carro, dandole un peso de 1, para que todos tengan el mismo peso
    # Y partiendo de la posicion 17, 25
    vehiculos.append([1, {"path": bfs(road_id, parking_map, 17, 25, parking_id[i+1], parking_position)}])

# Creamos la simulacion con todos los vehiculos
# El vehicle rate nos ayuda a aumentar o disminuir la separacion de los carros
sim.create_gen({
    # Se recomienda 100, maximo 300
    'vehicle_rate': 100,
    'vehicles': vehiculos
})


# Start simulation
win = Window(sim)
win.offset = (0, 0)
win.run(steps_per_update=1)