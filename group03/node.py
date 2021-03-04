import math

class Node:

    def __init__(self, id):
        self.nodeID = id
        self.costSoFar = math.inf
        self.estCost = math.inf
        self.parent = None


    def setCostSoFar(self, cost):
        self.costSoFar = cost

    def setEstCost(self, cost):
        self.estCost = cost

    def setParent(self, parent):
        self.parent = parent

    def getCostSoFar(self):
        return self.costSoFar

    def getEstCost(self):
        return self.estCost

    def getNodeId(self):
        return self.nodeID

    def getParent(self):
        return self.parent

    def __lt__(self, other):
        return self.estCost < other.estCost