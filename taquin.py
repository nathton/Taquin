import math
import time
from collections import namedtuple


P1 = {'0': 36, '1': 12, '2': 12, '3': 4, '4': 3, '5': 3, '6': 4, '7': 1}
P2 = P3 = {'0': 8, '1': 7, '2': 6, '3': 5, '4': 4, '5': 3, '6': 2, '7': 1}
P4 = P5 = {'0': 8, '1': 7, '2': 6, '3': 5, '4': 3, '5': 2, '6': 4, '7': 1}
P6 = {'0': 1, '1': 1, '2': 1, '3': 1, '4': 1, '5': 1, '6': 1, '7': 1}
P7 = {'0': 21, '1': 18, '2': 18, '3': 9, '4': 3, '5': 3, '6': 9, '7': 1}
RO1 = RO3 = RO5 = RO7 = 4
RO2 = RO4 = RO6 = 1


def calcul(point_A, point_B):
    return abs(point_B.x - point_A.x) + abs(point_B.y - point_A.y)

def minimum_moves(current_taquin, target_taquin):
    dimension = current_taquin.dimension
    min_moves = {}
    Point = namedtuple('Point', ['x', 'y'])
    for index, value in enumerate(current_taquin.state):
        value_x_origin = index % dimension
        value_y_origin = index // dimension
        value_x_goal = target_taquin.state.index(value) % dimension
        value_y_goal = target_taquin.state.index(value) // dimension
        origin_point = Point(value_x_origin, value_y_origin)
        goal_point = Point(value_x_goal, value_y_goal)
        min_moves[value] = calcul(origin_point, goal_point)
    return min_moves

def h1(current_taquin, target_taquin):
    distances_elem = minimum_moves(current_taquin, target_taquin)
    return sum(P1[i] * distances_elem[target_taquin.state[int(i)]] for i in P1)//RO1

def h2(current_taquin, target_taquin):
    distances_elem = minimum_moves(current_taquin, target_taquin)
    return sum(P2[i] * distances_elem[target_taquin.state[int(i)]] for i in P2)//RO2

def h3(current_taquin, target_taquin):
    distances_elem = minimum_moves(current_taquin, target_taquin)
    return sum(P3[i] * distances_elem[target_taquin.state[int(i)]] for i in P3)//RO3

def h4(current_taquin, target_taquin):
    distances_elem = minimum_moves(current_taquin, target_taquin)
    return sum(P4[i] * distances_elem[target_taquin.state[int(i)]] for i in P4)//RO4         

def h5(current_taquin, target_taquin):
    distances_elem = minimum_moves(current_taquin, target_taquin)
    return sum(P5[i] * distances_elem[target_taquin.state[int(i)]] for i in P5)//RO5         

def h6(current_taquin, target_taquin):
    distances_elem = minimum_moves(current_taquin, target_taquin)
    return sum(P6[i] * distances_elem[target_taquin.state[int(i)]] for i in P6)//RO6         

def h7(current_taquin, target_taquin):
    distances_elem = minimum_moves(current_taquin, target_taquin)
    return sum(P7[i] * distances_elem[target_taquin.state[int(i)]] for i in P7)//RO7  

def h8(current_taquin, target_taquin):
    if current_taquin.state == target_taquin.state:
        return 0
    else:
        return sum((i != j) for i, j in zip(current_taquin.state, target_taquin.state))


class Taquin():
    def __init__(self, state):
        self.state = state
        self.dimension = int(math.sqrt(len(state)))

    def move(self, direction):
        empty_coords = self.get_value_coords('X')
        new_state = list(self.state)
        if direction == 'E' and empty_coords.x < self.dimension - 1:
            new_state[empty_coords.index], new_state[empty_coords.index + 1] = new_state[empty_coords.index + 1], new_state[empty_coords.index]
        elif direction == 'O' and empty_coords.x > 0:
            new_state[empty_coords.index], new_state[empty_coords.index - 1] = new_state[empty_coords.index - 1], new_state[empty_coords.index]
        elif direction == 'N' and empty_coords.y > 0:
            new_state[empty_coords.index], new_state[empty_coords.index - self.dimension] = new_state[empty_coords.index - self.dimension], new_state[empty_coords.index]
        elif direction == 'S' and empty_coords.y < self.dimension - 1:
            new_state[empty_coords.index], new_state[empty_coords.index + self.dimension] = new_state[empty_coords.index + self.dimension], new_state[empty_coords.index]
        return Taquin(new_state)

    def get_value_coords(self, value):
        index = self.state.index(value)
        Points = namedtuple('Points', ['value', 'index', 'x', 'y'])
        return Points(value, index, index % self.dimension, index // self.dimension)

    def __repr__(self):
        two_dimension_list = [self.state[i:i+self.dimension] for i in range(0, len(self.state), self.dimension)]
        repr_str = ""
        for j in two_dimension_list :
            line = ("  |  ".join(str(i) for i in j))
            repr_str += line + '\n'
        return repr_str

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.state == other.state
        return NotImplemented 


class Node():
    def __init__(self, taquin, parent_node=None, move=''):
        self.taquin = taquin
        self.parent_node = parent_node
        self.cost = 0 if parent_node is None else parent_node.cost + 1
        self.move = move

    def child(self, direction):
        return Node(self.taquin.move(direction), self, direction)

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.taquin == other.taquin
        return NotImplemented

    def __repr__(self):
        return str(self.taquin)


class GraphConf():
    def __init__(self, initial_state, target_state, heuristic_fct):
        self.initial_taquin = Taquin(initial_state)
        self.target_taquin = Taquin(target_state)
        self.heuristic_fct = heuristic_fct

    def solve(self):
        if not self.check_validity():
            raise ValueError('Your input cannot be solved.')
        self.run_a_star_algo()

    def nb_inversions(self):
        dimension = self.initial_taquin.dimension
        init_state_without_empty = self.initial_taquin.state[:]
        init_state_without_empty.remove('X')
        target_state_without_empty = self.target_taquin.state[:]
        target_state_without_empty.remove('X')
        count = 0
        for index_i, value_i in enumerate(init_state_without_empty):
            index_t = target_state_without_empty.index(value_i)
            for j in target_state_without_empty[:index_t]:
                if j in init_state_without_empty[index_i:]:
                    count += 1
        return count

    def check_validity(self):
        parity_puzzle = self.initial_taquin.dimension % 2
        parity_inversions = self.nb_inversions() % 2
        if parity_puzzle != 0:
            return parity_inversions == 0
        else:
            empty_coords_in_init = self.initial_taquin.get_empty_coords()
            y_parity = empty_coords_in_init.y % 2
            return parity_inversions != y_parity
            
    def evaluate(self, node):
        return node.cost + self.heuristic_fct(node.taquin, self.target_taquin)

    def run_a_star_algo(self):
        t = time.process_time()
        nb_analyzed_nodes = 0
        initial_node = Node(self.initial_taquin)
        open_l = []
        close_l = []
        open_l.append(initial_node)
        while True:
            if open_l == []:
                raise ValueError('There is no solutions')
            else:
                current_node = open_l.pop(0)
                close_l.append(current_node)
                if current_node.taquin == self.target_taquin:
                    elapsed_time  = time.process_time() - t
                    node = current_node
                    results = [current_node]
                    chemin = [current_node.move]
                    while node.parent_node != None:
                        results.append(node.parent_node)
                        chemin.append(node.parent_node.move)
                        node = node.parent_node
                    chemin = ''.join(list(reversed(chemin)))
                    results = list(reversed(results))
                    print('===================================')
                    print('=============RESULTATS=============')
                    print('===================================')
                    print('ETATS INTERMEDIAIRES:')
                    for i in results:
                        print(i)
                    print('HEURISTIQUE UTILISEE : {}'.format(self.heuristic_fct.__name__))
                    print('TEMPS ECOULE : {}'.format(elapsed_time))
                    print("NOMBRE D'ETATS VISITES : {}".format(nb_analyzed_nodes))
                    print("CHEMIN EMPRUNTE : {}".format(chemin))
                    print("LONGUEUR DU CHEMIN : {}".format(len(chemin)))
                    return chemin
                else:
                    for direction in ['E', 'O', 'N', 'S']:
                        child_node = current_node.child(direction)                         
                        if not current_node.parent_node == child_node:
                            nb_analyzed_nodes += 1
                            if child_node in open_l:
                                f_child = self.evaluate(child_node)
                                child_twin = open_l[open_l.index(child_node)]
                                if f_child <= self.evaluate(child_twin):
                                    open_l.remove(child_twin)
                                    open_l.append(child_node)
                                    open_l = sorted(open_l, key=self.evaluate)                                    
                            elif child_node in close_l:
                                f_child = self.evaluate(child_node)
                                child_twin = close_l[close_l.index(child_node)]
                                if f_child <= self.evaluate(child_twin):
                                    close_l.remove(child_twin)
                                    open_l.append(child_node)
                                    open_l = sorted(open_l, key=self.evaluate)
                            else:
                                open_l.append(child_node)
                                open_l = sorted(open_l, key=self.evaluate)                


def main():
    print('This program can only solved 8-puzzle problems\n')
    print('Empty block is displayed as an X character\n')    
    initial_state = input('Please input the INITIAL state. Allowed format is : 0 1 2 3 4 X 5 6 7. X is the empty block\n')
    if not initial_state:
        raise ValueError('Initiial state cannot be None')
    parsed_init = initial_state.strip().split(' ')
    target_state = input('Please input the TARGET state. Allowed format is : 0 1 2 3 4  5 6 7 X. X is the empty block\n')
    if not target_state:
        raise ValueError('Target state cannot be None')    
    parsed_target = target_state.strip().split(' ')
    if ((len(parsed_init) != len(parsed_target)) or (len(parsed_init) != 9)):
        raise ValueError('Initiial state and Target state does not have the same size, or are not of size 9')
    heuristic_fct = input('choose from h1, h2, h3, h4, h5, h6, h7, h8\n')
    mapping_functions = {
    'h1': h1,
    'h2': h2,
    'h3': h3,
    'h4': h4,
    'h5': h5,
    'h6': h6,
    'h7': h7,
    'h8': h8
    }
    GraphConf(parsed_init, parsed_target, mapping_functions[heuristic_fct]).solve()

main()