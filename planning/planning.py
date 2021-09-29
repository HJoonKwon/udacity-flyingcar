from enum import Enum
from queue import PriorityQueue
import numpy as np 
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree


def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    if path is not None:
        pruned_path = [p for p in path]
        # TODO: prune the path!
        i = 0 
        while i < len(pruned_path) - 2:
            p1 = point(pruned_path[i])
            p2 = point(pruned_path[i+1])
            p3 = point(pruned_path[i+2]) 
            if collinearity_check(p1, p2, p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1             
    else:
        pruned_path = path
        
    return pruned_path

class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    UPRIGHT = (-1, 1, np.sqrt(2))
    UPLEFT = (-1, -1, np.sqrt(2))
    DOWNRIGHT = (1, 1, np.sqrt(2))
    DOWNLEFT = (1, -1, np.sqrt(2))

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
        elif self == self.UPRIGHT:
            return 'd'
        elif self == self.UPLEFT:
            return 'd'
        elif self == self.DOWNRIGHT:
            return 'd'
        elif self == self.DOWNLEFT:
            return 'd'    

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])

def collides(obstacle, point):    
    p1 = Point(point[0], point[1])
    polygon, height = obstacle
    if polygon.contains(p1)and height >= point[2]:
        return True 
    
    return False

class Sampler():
    
    def __init__(self, data):
        
        self._data = data 
        self._x_min = np.min(data[:,0] - data[:,3])
        self._x_max = np.max(data[:,0] + data[:,3])
        self._y_min = np.min(data[:,1] - data[:,4])
        self._y_max = np.max(data[:,1] + data[:,4])
        
        self._z_min = 0
        self._z_max = 20 
        
        self._random_seed = 0 
        
        self._polygons = []
        self._centers_of_polygons = [] 
        
        self._collision_radius = 2* np.max ((data[:,3], data[:,4]))
        
    
    def load_data(self, data):
        
        self._data = data 
        
    def sample(self, n_samples):
        
        np.random.seed(self._random_seed)
        x_vals = np.random.uniform(self._x_min, self._x_max, n_samples)
        y_vals = np.random.uniform(self._y_min, self._y_max, n_samples)
        z_vals = np.random.uniform(self._z_min, self._z_max, n_samples)
        
        samples = list(zip(x_vals, y_vals, z_vals))
        
        # make a KDtree with center points of polygons
        tree = KDTree(self._centers_of_polygons)
        
        valid_samples = [] 
        
        for sample in samples:
            in_collision = False 
            x,y,z = sample 
            idxs =  list(tree.query_radius(np.array([x, y]).reshape(1, -1), r=self._collision_radius)[0])
            if len(idxs)>0:
                for i in idxs:
                    if collides(self._polygons[i], sample):
                        in_collision = True 
                        break 
            if not in_collision:
                valid_samples.append(sample)        
            
        return valid_samples         

    def extract_polygons(self):
        
        polygons = [] 
        for i in range(self._data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = self._data[i,:]
            
            corners = [
                (north-d_north, east-d_east),
                (north-d_north, east+d_east),
                (north+d_north, east+d_east),
                (north+d_north, east-d_east)
            ]
            
            height = alt + d_alt 
            
            p = Polygon(corners)
            polygons.append((p, height))
            
        self._centers_of_polygons = np.array([(p[0].centroid.x, p[0].centroid.y) for p in polygons])
            
        self._polygons = polygons
            
        return polygons     
            


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN, Action.UPRIGHT, Action.UPLEFT, Action.DOWNRIGHT, 
            Action.DOWNLEFT]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)
    if x - 1 < 0 or y + 1 > m or grid[x-1, y+1] == 1:
        valid.remove(Action.UPRIGHT)
    if x - 1 < 0 or y - 1 < 0 or grid[x-1, y-1] == 1:
        valid.remove(Action.UPLEFT)
    if x + 1 > n or y + 1 > m or grid[x+1, y+1] == 1:
        valid.remove(Action.DOWNRIGHT)
    if x + 1 > n or y - 1 < 0 or grid[x+1, y-1] == 1:
        valid.remove(Action.DOWNLEFT)    

    return valid


def a_star_graph(graph, heuristic, start, goal):
    
    """Modified A* to work with NetworkX graphs."""

    # list - nodes of path (found optimal path)
    # queue - total cost(branch cost + heuristic cost) and current expandible nodes  
    # set - visited nodes 
    # dictionary - next node, corresponding current node and branch cost so far 
    
    path = []
    path_cost = 0 
    queue = PriorityQueue()
    visited = set(start)
    branch = {} 
    
    queue.put((0, start))
    
    is_found = False 
    
    while not queue.empty():
        
        item = queue.get()
        current_node = item[1]
        
        if current_node == start:
            current_cost = 0
        else:
            current_cost = branch[current_node][0]
            
        if current_node == goal:
            is_found = True 
            break 
        
        else:     
                
            for next_node in graph[current_node]:
                
                branch_cost = current_cost + graph.edges[current_node, next_node]['weight']
                heuristic_cost = heuristic(next_node, goal)
                total_cost = branch_cost + heuristic_cost
                
                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((total_cost, next_node))
                    
    
    if is_found:
        
        path_cost = branch[goal][0]
        path.append(goal)
        n = goal 
        while branch[n][1] is not start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])        
                
    else:
        print("Failed to find a path ")                
    
    return path[::-1], path_cost 


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

