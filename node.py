import numpy as np

class Node:
    def __init__(self, x, y, z, parent=None, cost=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent
        self.cost = cost

    def copy(self):
        return Node(self.x, self.y, self.z, self.parent)

    def to_array(self):
        return np.array([self.x, self.y, self.z])

    def __add__(self, rhs):
        return Node(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    
    def __sub__(self, rhs):
        return Node(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    
    def __mul__(self, rhs):
        if (not isinstance(rhs, Node)):
            # scalar multiplication
            return Node(rhs*self.x, rhs*self.y, rhs*self.z)
        else:
            return NotImplementedError("No vector multiplication needed")

    def __rmul__(self, rhs):
        return self * rhs

    def __abs__(self):
        return Node(abs(self.x), abs(self.y), abs(self.z))

    # def __eq__(self, rhs):
    #     return ((self.x == rhs.x) and ((self.y == rhs.y) and (self.z == rhs.z)))
    @staticmethod
    def normalize(node):
        x, y, z = node.x, node.y, node.z
        magnitude = (x**2 + y**2 + z**2)**(0.5)
        return Node(x/magnitude, y/magnitude, z/magnitude)

    def __str__(self):
        return self.__repr__()
    
    def __repr__(self):
        return (f"[{self.x}, {self.y}, {self.z}] cost = {self.cost}")

if __name__ == "__main__":
    # Run this to unit test this class

    node = Node(4, 5, 6)

    print("Testing adding node objects:")
    ans = Node(1, 2, 3) + node
    print(f"{Node(1, 2, 3)} + {node} = {ans}")

    print("Testing subtracting node objects:")
    ans = Node(1, 2, 3) - node
    print(f"{Node(1, 2, 3)} - {node} = {ans}")

    print("Testing scalar multiplication:")
    ans = 3 * node
    print(f"{3} * {node} = {ans}")