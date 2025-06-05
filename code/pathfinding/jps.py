import math, time, heapq


class JPS:

    def __init__(self):
        pass

    def heuristic(self, a, b, hchoice):
        if hchoice == 1:
            xdist = math.fabs(b[0] - a[0])
            ydist = math.fabs(b[1] - a[1])
            if xdist > ydist:
                return 14 * ydist + 10 * (xdist - ydist)
            else:
                return 14 * xdist + 10 * (ydist - xdist)
        if hchoice == 2:
            return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def blocked(self, cX, cY, dX, dY, matrix):
        if cX + dX < 0 or cX + dX >= matrix.shape[0]:
            return True
        if cY + dY < 0 or cY + dY >= matrix.shape[1]:
            return True
        if dX != 0 and dY != 0:
            if matrix[cX + dX][cY] == 1 and matrix[cX][cY + dY] == 1:
                return True
            if matrix[cX + dX][cY + dY] == 1:
                return True
        else:
            if dX != 0:
                if matrix[cX + dX][cY] == 1:
                    return True
            else:
                if matrix[cX][cY + dY] == 1:
                    return True
        return False

    def dblock(self, cX, cY, dX, dY, matrix):
        if matrix[cX - dX][cY] == 1 and matrix[cX][cY - dY] == 1:
            return True
        else:
            return False

    def direction(self, cX, cY, pX, pY):
        dX = int(math.copysign(1, cX - pX))
        dY = int(math.copysign(1, cY - pY))
        if cX - pX == 0:
            dX = 0
        if cY - pY == 0:
            dY = 0
        return (dX, dY)

    def nodeNeighbours(self, cX, cY, parent, matrix):
        neighbours = []
        if type(parent) != tuple:
            # Check if it's the starting point
            if parent is None:
                # Add only the neighbor in the direction of increasing y
                if not self.blocked(cX, cY, 0, 1, matrix):
                    neighbours.append((cX, cY + 1))
                return neighbours

        if type(parent) != tuple:
            for i, j in [
                (-1, 0),
                (0, -1),
                (1, 0),
                (0, 1),
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            ]:
                if not self.blocked(cX, cY, i, j, matrix):
                    neighbours.append((cX + i, cY + j))

            return neighbours
        dX, dY = self.direction(cX, cY, parent[0], parent[1])

        if dX != 0 and dY != 0:
            if not self.blocked(cX, cY, 0, dY, matrix):
                neighbours.append((cX, cY + dY))
            if not self.blocked(cX, cY, dX, 0, matrix):
                neighbours.append((cX + dX, cY))
            if (
                    not self.blocked(cX, cY, 0, dY, matrix)
                    or not self.blocked(cX, cY, dX, 0, matrix)
            ) and not self.blocked(cX, cY, dX, dY, matrix):
                neighbours.append((cX + dX, cY + dY))
            if self.blocked(cX, cY, -dX, 0, matrix) and not self.blocked(
                    cX, cY, 0, dY, matrix
            ):
                neighbours.append((cX - dX, cY + dY))
            if self.blocked(cX, cY, 0, -dY, matrix) and not self.blocked(
                    cX, cY, dX, 0, matrix
            ):
                neighbours.append((cX + dX, cY - dY))

        else:
            if dX == 0:
                if not self.blocked(cX, cY, dX, 0, matrix):
                    if not self.blocked(cX, cY, 0, dY, matrix):
                        neighbours.append((cX, cY + dY))
                    if self.blocked(cX, cY, 1, 0, matrix):
                        neighbours.append((cX + 1, cY + dY))
                    if self.blocked(cX, cY, -1, 0, matrix):
                        neighbours.append((cX - 1, cY + dY))

            else:
                if not self.blocked(cX, cY, dX, 0, matrix):
                    if not self.blocked(cX, cY, dX, 0, matrix):
                        neighbours.append((cX + dX, cY))
                    if self.blocked(cX, cY, 0, 1, matrix):
                        neighbours.append((cX + dX, cY + 1))
                    if self.blocked(cX, cY, 0, -1, matrix):
                        neighbours.append((cX + dX, cY - 1))
        return neighbours

    def jump(self, cX, cY, dX, dY, matrix, goal):

        nX = cX + dX
        nY = cY + dY
        if self.blocked(nX, nY, 0, 0, matrix):
            return None

        if (nX, nY) == goal:
            return (nX, nY)

        oX = nX
        oY = nY

        if dX != 0 and dY != 0:
            while True:
                if (
                        not self.blocked(oX, oY, -dX, dY, matrix)
                        and self.blocked(oX, oY, -dX, 0, matrix)
                        or not self.blocked(oX, oY, dX, -dY, matrix)
                        and self.blocked(oX, oY, 0, -dY, matrix)
                ):
                    return (oX, oY)

                if (
                        self.jump(oX, oY, dX, 0, matrix, goal) != None
                        or self.jump(oX, oY, 0, dY, matrix, goal) != None
                ):
                    return (oX, oY)

                oX += dX
                oY += dY

                if self.blocked(oX, oY, 0, 0, matrix):
                    return None

                if self.dblock(oX, oY, dX, dY, matrix):
                    return None

                if (oX, oY) == goal:
                    return (oX, oY)
        else:
            if dX != 0:
                while True:
                    if (
                            not self.blocked(oX, nY, dX, 1, matrix)
                            and self.blocked(oX, nY, 0, 1, matrix)
                            or not self.blocked(oX, nY, dX, -1, matrix)
                            and self.blocked(oX, nY, 0, -1, matrix)
                    ):
                        return (oX, nY)

                    oX += dX

                    if self.blocked(oX, nY, 0, 0, matrix):
                        return None

                    if (oX, nY) == goal:
                        return (oX, nY)

            else:
                while True:
                    if (
                            not self.blocked(nX, oY, 1, dY, matrix)
                            and self.blocked(nX, oY, 1, 0, matrix)
                            or not self.blocked(nX, oY, -1, dY, matrix)
                            and self.blocked(nX, oY, -1, 0, matrix)
                    ):
                        return (nX, oY)

                    oY += dY

                    if self.blocked(nX, oY, 0, 0, matrix):
                        return None

                    if (nX, oY) == goal:
                        return (nX, oY)

        return self.jump(nX, nY, dX, dY, matrix, goal)

    def identifySuccessors(self, cX, cY, came_from, matrix, goal):
        successors = []
        neighbours = self.nodeNeighbours(cX, cY, came_from.get((cX, cY), 0), matrix)

        for cell in neighbours:
            dX = cell[0] - cX
            dY = cell[1] - cY

            jumpPoint = self.jump(cX, cY, dX, dY, matrix, goal)

            if jumpPoint != None:
                successors.append(jumpPoint)

        return successors

    def method(self, matrix, start, goal, hchoice):

        came_from = {}
        close_set = set()
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal, hchoice)}

        pqueue = []

        heapq.heappush(pqueue, (fscore[start], start))

        starttime = time.time()

        while pqueue:

            current = heapq.heappop(pqueue)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.append(start)
                data = data[::-1]
                endtime = time.time()
                # print(gscore[goal])
                # return (data, round(endtime - starttime, 6))
                return data

            close_set.add(current)

            successors = self.identifySuccessors(
                current[0], current[1], came_from, matrix, goal
            )

            for successor in successors:
                jumpPoint = successor

                if (
                        jumpPoint in close_set
                ):  # and tentative_g_score >= gscore.get(jumpPoint,0):
                    continue

                tentative_g_score = gscore[current] + self.lenght(
                    current, jumpPoint, hchoice
                )

                if tentative_g_score < gscore.get(
                        jumpPoint, 0
                ) or jumpPoint not in [j[1] for j in pqueue]:
                    came_from[jumpPoint] = current
                    gscore[jumpPoint] = tentative_g_score
                    fscore[jumpPoint] = tentative_g_score + self.heuristic(
                        jumpPoint, goal, hchoice
                    )
                    heapq.heappush(pqueue, (fscore[jumpPoint], jumpPoint))
            endtime = time.time()
        return 0

    def lenght(self, current, jumppoint, hchoice):
        dX, dY = self.direction(current[0], current[1], jumppoint[0], jumppoint[1])
        dX = math.fabs(dX)
        dY = math.fabs(dY)
        lX = math.fabs(current[0] - jumppoint[0])
        lY = math.fabs(current[1] - jumppoint[1])
        if hchoice == 1:
            if dX != 0 and dY != 0:
                lenght = lX * 14
                return lenght
            else:
                lenght = (dX * lX + dY * lY) * 10
                return lenght
        if hchoice == 2:
            return math.sqrt(
                (current[0] - jumppoint[0]) ** 2 + (current[1] - jumppoint[1]) ** 2
            )
