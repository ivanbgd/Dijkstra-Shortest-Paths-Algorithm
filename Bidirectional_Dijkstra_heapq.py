"""
    *** Bidirectional Dijkstra Shortest Paths Algorithm ***

Useful for real road network graph problems, on a planar map/grid.

n - number of nodes 
m - number of edges

If the graph isn't dense, ie. it's sparse, it's better to implement
priority queue as heap than as array.
A graph is sparse when n and m are of the same order of magnitude.

Here, priority queue is implemented by using module heapq.

We put (dist, count, name) into heap, but count is not needed.

This version of the algorithm doesn't reconstruct the shortest path.
It only computes its length and returns it.

A lot faster if proc and procR are sets, than if they are lists.
Memory consumption is the same in both cases.
"""


import sys
import heapq
import itertools


class BidirectionalDijkstra:
    def __init__(self, n, adj, cost):
        self.n = n;                                     # Number of nodes
        self.adj = adj
        self.cost = cost
        self.inf = n*10**6                              # We'll consider all distances in the graph to be smaller.
        self.distance_ = [[self.inf]*n, [self.inf]*n]   # Initialize distances for forward and backward searches
        #self.proc = []                                 # closed set - forward
        #self.procR = []                                # closed set - backward
        self.proc = set()
        self.procR = set()
        self.counter = itertools.count()                # unique sequence count - not really needed here, but kept for generality
        self.valid = [[True] * n, [True] * n]           # is vertex (name) valid or not - it's valid while name (vertex) is in open set (in heap)

    def clear(self):
        """Reinitialize the data structures for the next query after the previous query."""
        self.distance_ = [[self.inf]*n, [self.inf]*n]
        #self.proc[:] = []
        #self.procR[:] = []
        self.proc.clear()
        self.procR.clear()
        self.valid = [[True] * n, [True] * n]

    def visit(self, q, side, v):
        """Try to relax the distance to node v from direction side by value dist."""
        i = 0   # needed for weights
        for neighbor in self.adj[side][v]:   # we relax all neighbor vertices (not edges); neighbor is "name" - 1
            # Relax(v, neighbor) - v is best.second
            if (self.distance_[side][neighbor] > self.distance_[side][v] + self.cost[side][v][i]):
                self.distance_[side][neighbor] = self.distance_[side][v] + self.cost[side][v][i];
                # We have to update open set with the new priority of neighbor.
                count = next(self.counter)
                entry = (self.distance_[side][neighbor], count, neighbor)
                heapq.heappush(q[side], entry)  # we're adding a new node to heap, but it's keeping the old name (neighbor); it also gets new count - this one has the smallest distance (priority), of all the nodes with this same name
            i += 1
        #self.proc.append(v) if side == 0 else self.procR.append(v)
        self.proc.add(v) if side == 0 else self.procR.add(v)


    def query(self, s, t):
        """ Returns the distance from s to t in the graph (-1 if there's no path). """
        self.clear()
        q = [[], []]  # q is a list of two priority queues (that are implemented as min-heaps); q[0] is forward, q[1] is reverse - those are the two "open" sets
        
        # Fill the open set (the heap).
        self.distance_[0][s] = 0
        self.distance_[1][t] = 0
        for i in range(self.n):
            # heap (pq) entry is (priority, count, task/key) == (distance, count, vertex name) - count is maybe not needed, but it's added for generality
            count = next(self.counter)
            entry = (self.distance_[0][i], count, i)
            heapq.heappush(q[0], entry)
            entry = (self.distance_[1][i], count, i)
            heapq.heappush(q[1], entry)

        while True:
            # the inner while loop removes and returns the best vertex
            best = None
            while q[0]:
                best = heapq.heappop(q[0])
                name = best[2]
                if self.valid[0][name]:
                    self.valid[0][name] = False;
                    break
            v = best[2]
            self.visit(q, 0, v)   # forward
            if v in self.procR:
                break

            # the inner while loop removes and returns the best vertex
            best = None
            while q[1]:
                best = heapq.heappop(q[1])
                name = best[2]
                if self.valid[1][name]:
                    self.valid[1][name] = False;
                    break
            v = best[2]
            self.visit(q, 1, v)   # backward
            if v in self.proc:
                break

        distance = self.inf

        # merge proc & procR
        #self.proc.extend(self.procR)        # lists - O(n) lookup
        self.proc = self.proc | self.procR  # sets - O(1) lookup
        
        for u in self.proc:
            if (self.distance_[0][u] + self.distance_[1][u] < distance):
                distance = self.distance_[0][u] + self.distance_[1][u]

        return distance if distance < self.inf else -1


if __name__ == '__main__':
    input = sys.stdin.read()                                    # Python 2; in console, after input, press Enter, then CTRL+Z, then Enter again
    data = list(map(int, input.split()))
    n, m = data[0:2]                                            # number of nodes, number of edges; nodes are numbered from 1 to n
    data = data[2:]
    adj = [[[] for _ in range(n)], [[] for _ in range(n)]]      # holds adjacency lists for every vertex in the graph
    cost = [[[] for _ in range(n)], [[] for _ in range(n)]]     # holds weights of the edges
    data = data[2*n:]
    # directed edge (u, v) of length c from the node number u to the node number v
    for e in range(m):
        u, v, c = data[3*e], data[3*e+1], data[3*e+2]
        adj[0][u-1].append(v-1)
        cost[0][u-1].append(c)
        adj[1][v-1].append(u-1)
        cost[1][v-1].append(c)
    data = data[3*m:]

    bidij = BidirectionalDijkstra(n, adj, cost)
    
    # the number of queries for computing the distance
    q = data[0]
    data = data[1:]
    # s and t are numbers ("names") of two nodes to compute the distance from s to t
    for i in range(q):
        s = data[2*i]
        t = data[2*i+1]
        print(bidij.query(s-1, t-1))

