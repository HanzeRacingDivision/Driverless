import json
class Quaternion():
    def __init__(self,a=1,b=0,c=0,d=0):
        self._a = a
        self._b = b
        self._c = c
        self._d = d

    def update(self,a,b,c,d):
        self._a = a
        self._b = b
        self._c = c
        self._d = d
    
    def _plus(self,q):
        a = self._a + q._a
        b = self._b + q._b
        c = self._c + q._c
        d = self._d + q._d
        return Quaternion(a,b,c,d)
    
    def _sub(self,q):
        a = self._a - q._a
        b = self._b - q._b
        c = self._c - q._c
        d = self._d - q._d
        return Quaternion(a,b,c,d)
    
    def _plus_scalar(self,s):
        a = self._a + s
        b = self._b + s
        c = self._c + s
        d = self._d + s
        return Quaternion(a,b,c,d)
    
    def _mult_scalar(self,s):
        a = self._a * s
        b = self._b * s
        c = self._c * s
        d = self._d * s
        return Quaternion(a,b,c,d)
    
    def _hamilton(self,q):
        a1 = self._a
        b1 = self._b
        c1 = self._c
        d1 = self._d

        a2 = q._a
        b2 = q._b
        c2 = q._c
        d2 = q._d

        a = a1*a2 - b1*b2 - c1*c2 - d1*d2
        b = a1*b1 + b1*a2 + c1*d2 - d1*c2
        c = a1*c2 - b1*b2 + c1*a2 + d1*b2
        d = a1*d2 + b1*c2 - c1*b2 + d1*a2
        return Quaternion(a,b,c,d)
    def conjugate(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        return Quaternion(a,-b,-c,-d)

    def unit(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        mag = (a**2+ b**2 + c**2 +d**2)**.5
        a/=mag
        b/=mag
        c/=mag
        d/=mag
        return Quaternion(a,b,c,d)
        
    def json(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d
        return json.dumps({'a':a,'b':b,'c':c,'d':d})

    def print(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        string= "a: {:.5f} b: {:.5f} c: {:5f} d: {:.5f}".format(a,b,c,d)
        print(string)
        
    def mag(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        return (a**2+ b**2 + c**2 +d**2)**.5

    def __add__(self,other):
        if type(other) == int or type(other) == float:
            return self._plus_scalar(other)
        if type(other) == Quaternion:
            return self._plus(other)
        raise TypeError
    
    def __sub__(self,other):
        if type(other) == int or type(other) == float:
            return self._plus_scalar(-other)
        if type(other) == Quaternion:
            return self._sub(other)
        raise TypeError
    
    
    def __mul__(self,other):
        if type(other) == int or type(other) == float:
            return self._mult_scalar(other)
        if type(other) == Quaternion:
            return self._hamilton(other)
        raise TypeError
    
    def __div__(self,other):
        if type(other) == int or type(other) == float:
            return self._mult_scalar(1./other)
        raise TypeError

        


def meters_to_degrees(meters):
    EARTH_RADIUS_M = 6371000
    PI = 3.1415926536
    #using small angle approximation sin theta ~= theta
    theta = meters/EARTH_RADIUS_M
    #Convert to degrees
    degrees = theta*360/PI
    return degrees