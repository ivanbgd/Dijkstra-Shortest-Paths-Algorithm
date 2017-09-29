"""
    *** Unidirectional Dijkstra Shortest Paths Algorithm ***

Useful for real road network graph problems, on a planar map/grid.

n - number of nodes 
m - number of edges

If the graph isn't dense, ie. it's sparse, it's better to implement
priority queue as heap than as array.
A graph is sparse when n and m are of the same order of magnitude.

Here, priority queue is implemented by using module heapq.

We put (dist, name) into heap; count is not needed.

This version of the algorithm doesn't reconstruct the shortest path.
It only computes its length and returns it.

A lot faster if we stop when name == t, than if we don't.
Memory consumption is the same in both cases.
"""


import sys
import heapq
#import itertools


class Dijkstra:
    def __init__(self, n, adj, cost):
        self.n = n;                                 # Number of nodes
        self.inf = n*10**6                          # We'll consider all distances in the graph to be smaller.
        self.distance = [self.inf]*n                # Initialize distances for forward search
        self.adj = adj
        self.cost = cost
        #self.counter = itertools.count()           # unique sequence count - not really needed here, but kept for generality
        self.valid = [True] * n                     # is vertex (name) valid or not - it's valid while name (vertex) is in open set (in heap)

    def query(self, s, t):
        """ Returns the distance from s to t in the graph (-1 if there's no path). """
        self.distance = [self.inf]*n
        self.valid = [True] * n
        self.distance[s] = 0
        
        # Fill the open set (the heap).
        open = []
        for i in range(self.n):
            # heap (pq) entry is (priority, count, task/key) == (distance, count, vertex name) - count is not needed, but it's added for generality
            #count = next(self.counter)
            #entry = (self.distance[i], count, i)
            entry = (self.distance[i], i)
            heapq.heappush(open, entry)

        # plain Dijkstra
        while open:
            # the inner while loop removes and returns the best vertex
            best = None
            name = None
            while open:
                best = heapq.heappop(open)
                name = best[-1]
                if self.valid[name]:
                    self.valid[name] = False;
                    break
            if name == t:
                break
            for i in range(len(self.adj[name])):                     # i is neighbor's index in adjacency list
                neighbor = adj[name][i]
                if self.distance[neighbor] > best[0] + self.cost[name][i]:        # best[0] == self.distance[best[-1]] == self.distance[name]
                    self.distance[neighbor] = best[0] + self.cost[name][i];
                    #count = next(self.counter)
                    #entry = (self.distance[neighbor], count, neighbor)
                    entry = (self.distance[neighbor], neighbor)
                    heapq.heappush(open, entry)

        return self.distance[t] if self.distance[t] < self.inf else -1


if __name__ == '__main__':
    input = sys.stdin.read()                                        # Python 2; in console, after input, press Enter, then CTRL+Z, then Enter again
    data = list(map(int, input.split()))
    n, m = data[0:2]                                                # number of nodes, number of edges; nodes are numbered from 1 to n
    data = data[2:]                                                 # edges
    data = data[2*n:]                                               # skip coordinates
    edges = list(zip(zip(data[0:(3 * m):3], data[1:(3 * m):3]), data[2:(3 * m):3]))
    adj = [[] for _ in range(n)]                                    # holds adjacency lists for every vertex in the graph
    cost = [[] for _ in range(n)]                                   # holds weights of the edges - since edges are here represented as starting from a node ("a"), and one node can have multiple edges, this is a list of lists, just like "adj"
    # directed edge (a, b) of length w from the node number a to the node number b
    for ((a, b), w) in edges:
        adj[a - 1].append(b - 1)
        cost[a - 1].append(w)
    data = data[3 * m:]

    dij = Dijkstra(n, adj, cost)

    # the number of queries for computing the distance
    q = data[0]
    data = data[1:]
    # s and t are numbers ("names") of two nodes to compute the distance from s to t
    for i in range(q):
        s = data[2*i]
        t = data[2*i+1]
        print(dij.query(s-1, t-1))

