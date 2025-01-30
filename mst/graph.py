import numpy as np
import heapq
from typing import Union

class Graph:

    def __init__(self, adjacency_mat: Union[np.ndarray, str]):
        """
    
        Unlike the BFS assignment, this Graph class takes an adjacency matrix as input. `adjacency_mat` 
        can either be a 2D numpy array of floats or a path to a CSV file containing a 2D numpy array of floats.

        In this project, we will assume `adjacency_mat` corresponds to the adjacency matrix of an undirected graph.
    
        """
        if type(adjacency_mat) == str:
            self.adj_mat = self._load_adjacency_matrix_from_csv(adjacency_mat)
        elif type(adjacency_mat) == np.ndarray:
            self.adj_mat = adjacency_mat
        else: 
            raise TypeError('Input must be a valid path or an adjacency matrix')
        
        
        self.mst = None

    def _load_adjacency_matrix_from_csv(self, path: str) -> np.ndarray:
        with open(path) as f:
            return np.loadtxt(f, delimiter=',')

    def construct_mst(self):
        """
    
        TODO: Given `self.adj_mat`, the adjacency matrix of a connected undirected graph, implement Prim's 
        algorithm to construct an adjacency matrix encoding the minimum spanning tree of `self.adj_mat`. 
            
        `self.adj_mat` is a 2D numpy array of floats. Note that because we assume our input graph is
        undirected, `self.adj_mat` is symmetric. Row i and column j represents the edge weight between
        vertex i and vertex j. An edge weight of zero indicates that no edge exists. 
        
        This function does not return anything. Instead, store the adjacency matrix representation
        of the minimum spanning tree of `self.adj_mat` in `self.mst`. We highly encourage the
        use of priority queues in your implementation. Refer to the heapq module, particularly the 
        `heapify`, `heappop`, and `heappush` functions.

        """

        if self.adj_mat.size == 0:
            raise ValueError('Adjacency matrix is empty, must have some vertices and edges')
                             
        if self.adj_mat is None:
            raise ValueError('Adjacency matrix is not initialized')


        if self.adj_mat.shape[0] != self.adj_mat.shape[1]:
            raise ValueError('Adjacency matrix must be symmetric and square')
        
        
        
        pq = [] # priority queue
        mst = []
        visited = set()
        start = np.random.randint(self.adj_mat.shape[0])
        visited.add(start)
        # add all neighbors of start to pq
        for i in range(self.adj_mat.shape[0]):
            if self.adj_mat[start][i] != 0: # if there is an edge
                heapq.heappush(pq, (self.adj_mat[start][i], start, i)) # tuple of (weight, start, neighbor)
        
        while pq:
            weight, start, neighbor = heapq.heappop(pq) # get the edge with the smallest weight
            if neighbor not in visited:
                visited.add(neighbor)
                mst.append((start, neighbor, weight))
                for i in range(self.adj_mat.shape[0]): #for all possible neighbors of neighbor
                    if self.adj_mat[neighbor][i] != 0: # if there is an edge
                        heapq.heappush(pq, (self.adj_mat[neighbor][i], neighbor, i))

        # create the mst matrix
        
        self.mst = np.zeros(self.adj_mat.shape)
        for start, neighbor, weight in mst:
            self.mst[start][neighbor] = weight
            self.mst[neighbor][start] = weight # symmetric

