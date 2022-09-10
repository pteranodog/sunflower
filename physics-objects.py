import time
import math

class Vector:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y
    
    @property
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    @property
    def sq_mag(self):
        return self.x ** 2 + self.y ** 2

    def add(self, vector):
        self.x += vector.x
        self.y += vector.y

    def sub(self, vector):
        self.x -= vector.x
        self.y -= vector.y

    def mult(self, scalar):
        self.x *= scalar
        self.y *= scalar

    def div(self, scalar):
        self.x /= scalar
        self.y /= scalar

    def floor_div(self, scalar):
        self.x //= scalar
        self.y //= scalar

    def dot(self, vector):
        pass # TODO

    def normalize(self):
        self.div(self.magnitude)
    
    def set_length(self, length):
        self.normalize()
        self.mult(length)

    def rotate(self):
        pass

    @staticmethod
    def add_(vector1, vector2):
        return Vector(vector1.x + vector2.x, vector1.y + vector2.y)

    @staticmethod
    def sub_(vector1, vector2):
        return Vector(vector1.x - vector2.x, vector1.y - vector2.y)
    
    @staticmethod
    def mult_(vector, scalar):
        return Vector(vector.x * scalar, vector.y * scalar)

    @staticmethod
    def div_(vector, scalar):
        return Vector(vector.x / scalar, vector.y / scalar)

    @staticmethod
    def floordiv_(vector, scalar):
        return Vector(vector.x // scalar, vector.y // scalar)

    def __add__(self, vector):
        return Vector.add2(self, vector)

    def __sub__(self, vector):
        return Vector.sub2(self, vector)

    def __mult__(self, scalar: float):
        return Vector.mult2(self, scalar)

    def __truediv__(self, scalar):
        return Vector(self.x / scalar, self.y / scalar)
    
    def __floordiv__(self, scalar):
        return Vector(self.x // scalar, self.y // scalar)
    
    def __str__(self) -> str:
        return f"({self.x},{self.y})"
    

class Object:
    def __init__(self, mass: float, moment_of_inertia: float, position: Vector, velocity: Vector, heading: float, rotation: float, attachments = []) -> None:
        self.mass = mass
        self.moment = moment_of_inertia
        self.pos = position
        self.vel = velocity
        self.heading = heading
        self.rot = rotation
        self.children = []
        self.parent = None
        self.fixed = False
        self.time_since_reset = time.time_ns()

    @property
    def abs_pos(self):
        if self.parent == None:
            return self.pos
        return self.parent.pos + self.pos

    def elapsed(self):
        return (time.time_ns() - self.last_updated) / 1e9
    
    def reset_elapsed(self):
        self.last_updated = time.time_ns()

    def attach(self, attachment) -> None:
        attachment.fixed = True
        attachment.parent = self
        self.attached.append(attachment)
    
    def update(self):
        if not self.fixed:
            self.pos.add(Vector.mult2(self.vel, self.elapsed()))
        self.reset_elapsed()