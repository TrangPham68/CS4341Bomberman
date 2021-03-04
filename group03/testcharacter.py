# This is necessary to find the main code
import math
import sys
from queue import PriorityQueue

import node

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here
        path = self.pathfinding((self.x, self.y), (5,17), wrld) #hard code end point
        print (self.x, " ", self.y)
        if (len(path) > 1):
            print(len(path))
            print (path[1][0] - self.x, " ", path[1][1] - self.y)
            self.move(path[1][0] - self.x, path[1][1] - self.y)

        #pass

    def pathfinding(self, start, end, world):  # start (x,y) and end (x,y)

        startNode = self.createNode(start)
        endNode = self.createNode(end)
        visited = {}

        frontier = PriorityQueue()
        startNode.setCostSoFar(0)
        startNode.setEstCost(self.getHeuristic(start, end))
        frontier.put((startNode.getEstCost(), startNode))

        while (not frontier.empty()):
            next = frontier.get()[1]
            visited[next.getNodeId()] = next

            if next.getNodeId() == endNode.getNodeId():
                return self.getPath(startNode, next)
            neighbor = self.getNeighbor(next.getNodeId(), world)
            for i in neighbor:
                # if wall, ignore
                node = self.createNode(i)
                if (world.wall_at(i[0], i[1])):
                    continue
                if (next.getParent() == None or not (node == next.getParent())):  # no going back
                    if (next.getCostSoFar() == math.inf):
                        next.setCostSoFar(0)
                    cost = self.getDistance(next.getNodeId(), node.getNodeId()) + next.getCostSoFar()

                    if (cost < node.getCostSoFar()):
                        node.setParent(next)
                        node.setCostSoFar(cost)
                        node.setEstCost(cost + self.getHeuristic(node.getNodeId(), endNode.getNodeId()))
                        frontier.put((node.getEstCost(), node))

        return []

    def getHeuristic(self, start, end):  # for now just compute distance
        return self.getDistance(start, end)  # TODO

    def getDistance(self, start, end):  # for now just compute distance
        return math.sqrt(math.pow(start[0] - end[0], 2) + math.pow(start[1] - end[1], 2))

    def getPath(self, startNode, endNode):
        current = endNode
        path = []

        while (not current == startNode):
            path.insert(0, current.getNodeId())
            current = current.getParent()

        path.insert(0, startNode.getNodeId())
        print(path)
        return path

    def getNeighbor(self, pos, world):
        width = world.width()
        height = world.height()
        neighbor = set()

        if (pos[0] > 0):
            neighbor.add((pos[0] - 1, pos[1]))  # left
            if (pos[1] > 0):
                neighbor.add((pos[0] - 1, pos[1] - 1))
            if (pos[1] < height - 1):
                neighbor.add((pos[0] - 1, pos[1] + 1))

        if (pos[0] < width - 1):
            neighbor.add((pos[0] + 1, pos[1]))  # right
            if (pos[1] > 0):
                neighbor.add((pos[0] + 1, pos[1] - 1))
            if (pos[1] < height - 1):
                neighbor.add((pos[0] + 1, pos[1] + 1))

        if (pos[1] > 0):
            neighbor.add((pos[0], pos[1] - 1))  # down

        if (pos[1] < height - 1):
            neighbor.add((pos[0], pos[1] + 1))
        return neighbor

    def createNode(self, pos):
        return node.Node(pos)




