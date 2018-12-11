import numpy as np
import copy
import time

class DuckieGraph(object):

    def __init__(self):
        
        # Graph Variables
        self.vertices = [] # List of n tuples
        self.edges = [] # List of lists of tuples
        
    
    def add_vertex(self, data=None):
        self.vertices.append(data)
        self.edges.append([])
        
    def add_edge(self, v1, v2, cmd, w=1.0):
        ''' --- Input ---
            v1 INT: idx of leaving vertex
            v2 INT: idx of destination vertex
            cmd STR: turn command ('L', 'R', 'F')
            w FLOAT: weight of path
            
            --- Important ---
            EDGE: (idx_v2 INT, cmd STR, w FLOAT)'''
            
        self.edges[v1].append((v2, cmd, w))
        
    def children(self, v):
        return self.edges[v]
        
    def clear(self):
        self.vertices = []
        self.edges = []
        
    def build_DTown(self, filepath):
        # Remove old stuff
        self.clear()

        # Open map file
        duck_map = open(filepath, 'r')
        line = duck_map.readline()
        while line != '':
            # Add Vertices
            v1_data = (int(line[0]), int(line[2]))
            v2_data = (int(line[2]), int(line[6]))
            if v1_data not in self.vertices:
                self.add_vertex(data=v1_data)
            if v2_data not in self.vertices:
                self.add_vertex(data=v2_data)
                
            # Add Edge
            idx_v1 = self.find_vertex(v1_data);
            idx_v2 = self.find_vertex(v2_data);
            self.add_edge(idx_v1, idx_v2, line[4], w=float(line[8:len(line)]))
            
            # Read next Line
            line = duck_map.readline()
            
        duck_map.close()
        
    def find_vertex(self, data):
        idx = -1
        if data in self.vertices:
            idx = self.vertices.index(data)
        return idx
        
class GraphSearch(object):

    def __init__(self):
        pass
        
    def build_frontier(self, children, curr_path, curr_cost):
        # Add new vertices as need to frontier
        for child in children:
            #print child
            #print curr_path
            if child[0] not in self.visited:
                total_cost = curr_cost + child[2]
                total_path = copy.deepcopy(curr_path)
                total_path.append(child[1])
                if self.frontier.has_key(child[0]):
                    if total_cost < self.frontier[child[0]][0]:
                        self.frontier[child[0]] = (total_cost, total_path)
                else:
                    self.frontier[child[0]] = (total_cost, total_path)
                    
        # Reorder the frontier largest to smallest
        temp_list = self.frontier.items()
        temp_list.sort(key=lambda x: x[1][0])
        self.frontier_keys = []
        for elem in temp_list:
            self.frontier_keys.insert(0, (elem[0], elem[1][0]))
        
        
    def shortest_path(self, start_idx, goal_int, graph):
    
        #Frontier elem: (idx: cost, [cmds])
        #Frontier keys: (idx, cost)
        self.visited = []
        self.frontier = {}
        self.frontier_keys = []
        curr_node = None
        curr_path = []
        curr_idx = start_idx
        self.visited.append(start_idx)
        children = graph.children(start_idx)
        self.build_frontier(children, curr_path, 0)
        while len(self.frontier) != 0:
            #print self.frontier
            curr_idx = self.frontier_keys.pop()[0]
            curr_node = self.frontier[curr_idx]
            self.frontier.pop(curr_idx)
            if graph.vertices[curr_idx][1] == goal_int:
                break
            self.visited.append(curr_idx)
            children = graph.children(curr_idx)
            self.build_frontier(children, curr_node[1], curr_node[0])
            
        return curr_node[1]





if __name__=="__main__":
    a = DuckieGraph()
    a.build_DTown()
    #print a.vertices
    #print a.edges
    b = GraphSearch()
    b.shortest_path(5, 4, a)





