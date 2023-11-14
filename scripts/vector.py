from __future__ import division
from collections import Sequence
from math import acos, atan2, cos, pi, sin, sqrt

__all__ = ['IncompatibleDimensions', 'Vector']

class IncompatibleDimensions(Exception):
    pass

class Vector(tuple):
    """Vector is a subclass of tuple representing a vector in two or more
    dimensions. You create a Vector by passing an iterable (that
    yields the elements of the vector) to the constructor:

        >>> Vector(range(5))
        Vector(0, 1, 2, 3, 4)

    or, for vectors of dimensions other than 1, by passing the
    elements as arguments to the constructor:

        >>> Vector(1, 2, 3)
        Vector(1, 2, 3)

    The first three components of a vector are accessible via the x,
    y, and z properties:

        >>> v = Vector(1, 2, 3)
        >>> v.x, v.y, v.z
        (1, 2, 3)

    The elements must support numeric operations, but otherwise
    a Vector is agnostic as to their type:

        >>> Vector(1.5, 2.5, 3.5)
        Vector(1.5, 2.5, 3.5)
        >>> from fractions import Fraction
        >>> Vector(Fraction(1, i) for i in (2, 3, 4))
        Vector(Fraction(1, 2), Fraction(1, 3), Fraction(1, 4))

    Vectors support vector addition and subtraction, and
    multiplication and division by scalars:

        >>> v, w = Vector(1, 2), Vector(3.5, 4.5)
        >>> v + w
        Vector(4.5, 6.5)
        >>> w - v
        Vector(2.5, 2.5)
        >>> v * 10
        Vector(10, 20)
        >>> 100 * v
        Vector(100, 200)
        >>> -v
        Vector(-1, -2)
        >>> +w
        Vector(3.5, 4.5)
        >>> w / 0.5
        Vector(7.0, 9.0)
        >>> v * 9 // 10
        Vector(0, 1)

    You can combine vectors and ordinary sequences:

        >>> Vector(0, 0) + (1, 2)
        Vector(1, 2)

    but the dimensions must match:

        >>> Vector(0, 0) + (0, 0, 0)
        ... # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        IncompatibleDimensions: (2, 3)

    The magnitude property of a vector gives its magnitude, as does the
    abs() function:

        >>> Vector(3, 4).magnitude
        5.0
        >>> abs(Vector(5, 12))
        13.0

    Note that the magnitude is always a float because of the square
    root. The magnitude_squared property gives the square of the
    magnitude, and this varies according to the type of the elements:

        >>> Vector(Fraction(1, 3), Fraction(1, 3)).magnitude_squared
        Fraction(2, 9)

    The angle_to() method computes the unsigned angle between two
    vectors:

        >>> from math import pi
        >>> Vector(1, 0, 0).angle_to((0, 1, 0)) / pi
        0.5

    The distance() method computes the distance between two points
    (expressed as vectors):

        >>> Vector(3, -5, 1).distance((1, -2, 7))
        7.0

    The dot() method computes the dot product.

        >>> Vector(1, 2, 3).dot((3, 2, 1))
        10

    The map() method applies a function to each element in the vector:

        >>> Vector(1.5, 2.6, 3.4).map(round)
        Vector(2, 3, 3)

    The scaled() method returns a vector in the same direction but
    with the specified magnitude; the normalized() method returns a unit
    vector in the same direction; and the projected() method projects
    another vector onto a vector.

        >>> Vector(10, 20, 20).scaled(3)
        Vector(1.0, 2.0, 2.0)
        >>> Vector(0, 0, 5).normalized()
        Vector(0.0, 0.0, 1.0)
        >>> Vector(2, 0, 0).projected((5, 6, 7))
        Vector(5.0, 0.0, 0.0)

    Any of the above three methods may raise ZeroDivisionError if
    applied to a vector with magnitude zero:

        >>> Vector(0, 0).scaled(3)
        ... # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        ZeroDivisionError: float division by zero
        >>> Vector(0, 0, 0).normalized()
        ... # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        ZeroDivisionError: float division by zero
        >>> Vector(0, 0, 0, 0).projected((1, 2, 3, 4))
        ... # doctest: +IGNORE_EXCEPTION_DETAIL
        Traceback (most recent call last):
            ...
        ZeroDivisionError: division by zero


    Three-dimensional vectors support one extra operation: the cross()
    method computes the cross product:

        >>> Vector(2, 1, 0).cross((3, 4, 0))
        Vector(0, 0, 5)


    Two-dimensional vectors support four extra operations.

    The angle property is the signed angle (in radians) that the
    vector makes with the x-axis:

        >>> Vector(1, -1).angle / pi
        -0.25

    The cross() method returns the signed magnitude of the cross
    product between the two vectors (interpreted as if they were
    three- dimensional vectors lying in the XY-plane):

        >>> Vector(3, 4).cross((2, 1))
        -5

    The perpendicular() method returns a vector that's perpendicular
    to the vector:

        >>> Vector(3, 4).perpendicular()
        Vector(-4, 3)

    The rotated() method rotates a vector through an angle in radians
    and returns the result:

        >>> Vector(2, 0).rotated(pi / 4)
        ... # doctest: +ELLIPSIS
        Vector(1.414213562373..., 1.414213562373...)

    """
    def __new__(cls, *args):
        if len(args) == 1: args = args[0]
        return super(Vector, cls).__new__(cls, tuple(args))

    def __repr__(self):
        if len(self) == 1:
            fmt = '{0}({1!r})'
        else:
            fmt = '{0}{1!r}'
        return fmt.format(type(self).__name__, tuple(self))

    def _check_compatibility(self, other):
        if len(self) != len(other):
            raise IncompatibleDimensions(len(self), len(other))

    def _dimension_error(self, name):
        return ValueError('.{0}() is not implemented for {1}-dimensional '
                          'vectors.'.format(name, len(self)))

    def __add__(self, other):
        if not isinstance(other, Sequence):
            return NotImplemented
        self._check_compatibility(other)
        return Vector(v + w for v, w in zip(self, other))

    def __radd__(self, other):
        if not isinstance(other, Sequence):
            return NotImplemented
        self._check_compatibility(other)
        return Vector(w + v for v, w in zip(self, other))

    def __sub__(self, other):
        if not isinstance(other, Sequence):
            return NotImplemented
        self._check_compatibility(other)
        return Vector(v - w for v, w in zip(self, other))

    def __rsub__(self, other):
        if not isinstance(other, Sequence):
            return NotImplemented
        self._check_compatibility(other)
        return Vector(w - v for v, w in zip(self, other))

    def __mul__(self, s):
        return Vector(v * s for v in self)

    def __rmul__(self, s):
        return Vector(v * s for v in self)

    def __div__(self, s):
        return Vector(v / s for v in self)

    def __truediv__(self, s):
        return Vector(v / s for v in self)

    def __floordiv__(self, s):
        return Vector(v // s for v in self)

    def __neg__(self):
        return self * -1

    def __pos__(self):
        return self

    def __abs__(self):
        return sqrt(self.magnitude_squared)

    @property
    def angle(self):
        """The signed angle [-pi, pi] between this vector and the x-axis. For
        two-dimensional vectors only.

        """
        if len(self) == 2:
            return atan2(self[1], self[0])
        else:
            raise self._dimension_error('angle')

    def angle_to(self, other):
        """Return the unsigned angle [0, pi] to another vector. If either
        vector has magnitude zero, raise ZeroDivisionError.

        """
        if len(self) in (2, 3):
            return atan2(abs(self.cross(other)), self.dot(other))
        else:
            # We avoid other.magnitude_squared so that this works if
            # other is a plain sequence rather than a Vector object.
            return acos(self.dot(other)
                        / sqrt(self.magnitude_squared
                               * sum(v * v for v in other)))

    def cross(self, other):
        """Return the cross product with another vector. For two-dimensional
        and three-dimensional vectors only.

        """
        self._check_compatibility(other)
        if len(self) == 2:
            return self[0] * other[1] - self[1] * other[0]
        elif len(self) == 3:
            return Vector(self[1] * other[2] - self[2] * other[1],
                          self[2] * other[0] - self[0] * other[2],
                          self[0] * other[1] - self[1] * other[0])
        else:
            raise self._dimension_error('cross')

    def distance(self, other):
        """Return the distance to another vector (understanding both vectors
        as points).

        """
        return abs(self - other)

    def dot(self, other):
        """Return the dot product with the other vector."""
        self._check_compatibility(other)
        return sum(v * w for v, w in zip(self, other))

    @property
    def is_zero(self):
        """True if this vector has magnitude zero, False otherwise."""
        return self.magnitude_squared == 0

    @property
    def magnitude(self):
        """The magnitude of the vector."""
        return abs(self)

    @property
    def magnitude_squared(self):
        """The squared magnitude of the vector."""
        return sum(v * v for v in self)

    def map(self, f):
        """Return the vector whose elements are the result of applying the
        function f to the elements of this vector.
        
        """
        return Vector(f(v) for v in self)

    @property
    def non_zero(self):
        """False if this vector has magnitude zero, True otherwise."""
        return self.magnitude_squared != 0

    def normalized(self):
        """Return a unit vector in the same direction as this vector. If this
        has magnitude zero, raise ZeroDivisionError.

        """
        return self / abs(self)
    
    def perpendicular(self):
        """Return a vector perpendicular to this vector. For two-dimensional
        vectors only.

        """
        if len(self) == 2:
            return Vector(-self[1], self[0])
        else:
            raise self._dimension_error('perpendicular')

    def projected(self, other):
        """Return the projection of another vector onto this vector. If this
        vector has magnitude zero, raise ZeroDivisionError.

        """
        return self * (self.dot(other) / self.magnitude_squared)

    def rotated(self, theta):
        """Return the vector rotated through theta radians about the
        origin. For two-dimensional vectors only.

        """
        if len(self) == 2:
            s, c = sin(theta), cos(theta)
            return Vector(self.dot((c, -s)), self.dot((s, c)))
        else:
            raise self._dimension_error('rotated')

    def scaled(self, s):
        """Return a vector of magnitude s in the same direction as this vector.
        If this has magnitude zero, raise ZeroDivisionError.

        """
        return self * (s / abs(self))

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def z(self):
        return self[2]
