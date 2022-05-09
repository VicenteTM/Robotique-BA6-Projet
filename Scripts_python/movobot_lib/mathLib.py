# EP2-MOVOBOT
# File responsible for the math computations used for the plot

# Libraries import
import math

def rect(r, theta):
    """theta in degrees

    returns tuple; (float, float); (x,y)
    """
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x,y


def polar(x, y):
    """returns r, theta(degrees)
    """
    r = (x ** 2 + y ** 2) ** .5
    theta = math.degrees(math.atan2(y,x))
    return r, theta


class Vector(object):
    def __init__(self, x=None, y=None, r=None, theta=None):
        """x and y or r and theta(degrees)
        """
        if x is not None and y is not None:
            self.c_polar(x, y)
        elif r is not None and theta is not None:
            self.c_rect(r, theta)
        else:
            raise ValueError('Must specify x and y or r and theta')

    def c_polar(self, x, y, f = polar):
        self._x = x
        self._y = y
        self._r, self._theta = f(self._x, self._y)
        self._theta_radians = math.radians(self._theta)

    def c_rect(self, r, theta, f = rect):
        """theta in degrees
        """
        self._r = r
        self._theta = theta
        self._theta_radians = math.radians(theta)
        self._x, self._y = f(self._r, self._theta)
        
    def setx(self, x):
        self.c_polar(x, self._y)
        
    def getx(self):
        return self._x
    x = property(fget = getx, fset = setx)
    
    def sety(self, y):
        self.c_polar(self._x, y)
        
    def gety(self):
        return self._y
    y = property(fget = gety, fset = sety)
    
    def setxy(self, x, y):
        self.c_polar(x, y)
        
    def getxy(self):
        return self._x, self._y
    xy = property(fget = getxy, fset = setxy)
    
    def setr(self, r):
        self.c_rect(r, self._theta)
        
    def getr(self):
        return self._r
    r = property(fget = getr, fset = setr)
    
    def settheta(self, theta):
        """theta in degrees
        """
        self.c_rect(self._r, theta)
        
    def gettheta(self):
        return self._theta
    theta = property(fget = gettheta, fset = settheta)
    
    def set_r_theta(self, r, theta):
        """theta in degrees
        """
        self.c_rect(r, theta)
        
    def get_r_theta(self):
        return self._r, self._theta
    r_theta = property(fget = get_r_theta, fset = set_r_theta)
    
    def __str__(self):
        return '({},{})'.format(self._r, self._theta)
        
    def __add__(self, other):
        sum_x = self._x + other.x
        sum_y = self._y + other.y
        return Vector(x = sum_x, y = sum_y)
        
    def __radd__(self, other):
        if other == 0:
            return self
        else:
            return self.__add__(other)