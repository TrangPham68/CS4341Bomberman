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
        print (self.x, " ", self.y)
        path = self.pathfinding((self.x, self.y), (6, 17), wrld) #hard code end point
        if (len(path) > 1):
            print(len(path))
            self.move(path[1][0] - self.x, path[1][1] - self.y)
        else:
            self.move(0,0)


        #pass


    def pathfinding(self, start, end, world):  # start (x,y) and end (x,y)
        """Apply Astar to find the closest path from start to end in world"""
        startNode = self.createNode(start)
        endNode = self.createNode(end)
        visited = {}

        seenNeighbor = {}

        frontier = PriorityQueue()
        startNode.setCostSoFar(0)
        startNode.setEstCost(self.getHeuristic(start, end))
        frontier.put(startNode)
        visited[start] = 0

        seenNeighbor[start] = startNode

        while (not frontier.empty()):
            next = frontier.get()
            #visited[next.getNodePos()] = next

            if next.getNodePos() == endNode.getNodePos():
                return self.getPath(startNode, next)
            neighbor = self.getNeighbor(next.getNodePos(), world)
            for i in neighbor:
                # if wall, ignore
                if (world.wall_at(i[0], i[1])):
                    continue

                if not i in seenNeighbor:
                    node = self.createNode(i)
                    seenNeighbor[i] = node
                else:
                    node = seenNeighbor.get(i)


                if (next.getParent() == None or not (node == next.getParent())):
                # if the node is not next parents -> avoid suplicate path
                    if (next.getCostSoFar() == math.inf):
                        next.setCostSoFar(0)
                    cost = self.getDistance(next.getNodePos(), node.getNodePos()) + next.getCostSoFar()

                    if (cost < node.getCostSoFar()):
                        node.setParent(next)
                        node.setCostSoFar(cost)
                        node.setEstCost(cost + self.getHeuristic(node.getNodePos(), endNode.getNodePos()))
                        frontier.put(node)

        return [] #return empty if we find no path

    def getHeuristic(self, start, end):  # for now just compute distance
        """return the heuristic value from start point to end point"""
        return self.getDistance(start, end)  # TODO

    def getDistance(self, start, end):  # compute distance
        """return linear distance from start to end point"""
        return math.sqrt(math.pow(start[0] - end[0], 2) + math.pow(start[1] - end[1], 2))

    def getPath(self, startNode, endNode):
        """return the list of (x, y) point from start to end by backtracking parent node from end"""
        current = endNode
        path = []

        while (not current == startNode):
            path.insert(0, current.getNodePos())
            current = current.getParent()

        path.insert(0, startNode.getNodePos())
        print(path)
        return path


    def getNeighbor(self, pos, world):
        """get all the possible neighbor location and return a set of (x,y) neighbors"""
        width = world.width()
        height = world.height()
        neighbor = []

        if (pos[0] > 0):
            neighbor.append((pos[0] - 1, pos[1]))  # left
            if (pos[1] > 0):
                neighbor.append((pos[0] - 1, pos[1] - 1)) #diag left down
            if (pos[1] < height - 1):
                neighbor.append((pos[0] - 1, pos[1] + 1)) #diag left up

        if (pos[0] < width - 1):
            neighbor.append((pos[0] + 1, pos[1]))  # right
            if (pos[1] > 0):
                neighbor.append((pos[0] + 1, pos[1] - 1)) #diag right down
            if (pos[1] < height - 1):
                neighbor.append((pos[0] + 1, pos[1] + 1)) #diag right up

        if (pos[1] > 0):
            neighbor.append((pos[0], pos[1] - 1))  # down

        if (pos[1] < height - 1):
            neighbor.append((pos[0], pos[1] + 1))
        return neighbor

    def createNode(self, pos):
        """create a Node of a specific location"""
        return node.Node(pos)





