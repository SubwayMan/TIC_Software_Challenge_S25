from collections import deque
class RobotPath():
    def __init__(self, edges=[]):
        self.edges = deque(edges)

    def next(self):
        self.edges.rotate(-1)
        return self.current()

    def current(self):
        return self.edges[0]

    def add_edge(self, start, end, stop_dist, rotation):
        self.edges.append(RobotPathEdge(start, end, stop_dist, rotation))

class RobotPathEdge():
    def __init__(self, tag1, tag2, stop_dist, rotation):
        self.start = tag1
        self.end = tag2
        self.stop_dist = stop_dist
        self.rotation = rotation
