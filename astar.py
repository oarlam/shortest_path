import math
# destination, road_length, x, y
roads = {'a':{('b', 1), ('c', 2)},
             'b': {('a', 0.9), ('d', 3.5)},
             'c': {('d', 1.5), ('a', 2)},
             'd': {('c', 1.5), ('b', 3.5)}
}
pos = {'a': (0, 0), 'b': (1, 1), 'c':(2.5, 4), 'd':(3.5, 4)}

def potential(node_x, node_y, end_x, end_y):
    return math.sqrt((node_x - end_x) ** 2 + (node_y - end_y) ** 2)

def a_star_method(all_roads, start, end):
    h_start = potential(pos[start][0], pos[start][1], pos[end][0], pos[end][1])
    values = {start: [0, h_start, h_start, None]}
    open_set = [start]
    closed_set = set()
    while len(open_set) > 0:
        open_set.sort(key = lambda x: values[x][2])
        current_node = open_set.pop(0)
        closed_set.add(current_node)
        if current_node == end:
            return values
        for city, lenght in all_roads[current_node]:
            if city in closed_set:
                continue
            g_value = values[current_node][0] + lenght
            h_value = potential(pos[city][0], pos[city][1], pos[end][0], pos[end][1])
            distance = g_value + h_value
            if city not in open_set:
                open_set += [city]
                values[city] = [g_value, h_value, distance, current_node]
            elif city in open_set and values[city][2] > distance:
                values[city] = [g_value, h_value, distance, current_node]


s = 'a'
e = 'd'
steps = a_star_method(roads, s, e)

def get_path(values, start, end):
    distance = values[end][0]
    path = [end]
    node = end
    while True:
        node = values[node][3]
        path += [node]
        if node == start:
            return path[::-1], distance

print(get_path(steps, s, e))
