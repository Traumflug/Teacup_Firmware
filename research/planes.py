#!/usr/bin/env python

# Experiments with coefficients of a geometric plane

# Resources:
# http://www.wolframalpha.com/input/?i=plane+through+(1,-2,0),(4,-2,-2),(4,1,4)&lk=3
#    ==> 2 x - 6 y + 3 z + 14 == 0


# Translate a point relative to some origin
from __future__ import print_function


def translate(point, origin):
    return tuple([a - b for a, b in zip(point, origin)])


# Given two points in 3d space, define a vector
def vector(p1, p2):
    return tuple([b - a for a, b in zip(p1, p2)])


# Given two vectors in a plane, find the normal vector
def normal(u, v):
    # A normal vector is the cross-product of two coplanar vectors
    return tuple(
        [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ]
    )


def plane_from_three_points(P, Q, R):
    u = vector(P, Q)
    v = vector(P, R)
    n = normal(u, v)

    # Find the coefficients
    (A, B, C) = n

    # The equation of the plane is thus Ax+By+Cz+K=0.
    # Solve for K to get the final coefficient
    (x, y, z) = P
    K = -(A * x + B * y + C * z)

    return (A, B, C, K)


# find the Z offset for any x,y
# z = -(Ax + By + K) / C
def calcz(x, y, plane, translation=(0, 0, 0)):
    (A, B, C, K) = plane
    (tx, ty, tz) = translation
    return -(A * (x - tx) + B * (y - ty) + K) / C + tz


# Verify a point is on this plane
def validate(plane, point):
    (A, B, C, K) = plane
    (x, y, z) = point
    return z == calcz(x, y, plane)


def verify_plane(points):
    print("  ", "\n   ".join([str(p) for p in points]))

    plane = plane_from_three_points(*points)
    print("Plane coordinates: ", plane)

    if plane[2] == 0:
        print("   Error: points are colinear")
        return

    valid = True
    for p in points:
        if not validate(plane, p):
            print("Failed: sample point not on plane, ", p)
            valid = False
    print("Validation:", "Failed" if not valid else "Passed")


samples = [
    # canonical example
    [(1, -2, 0), (4, -2, -2), (4, 1, 4)],
    # three colinear points (infinite planes)
    [(2, 2, 2), (4, 4, 4), (10, 10, 10)],
    # Extreme tilt example in mm
    [(57, 123, -5), (200, 0, 35), (0, 207, 2)],
    # Some more examples in um
    [(0, 0, 1300), (200000, 200000, 3500), (0, 150000, -1000)],
    [(20000, 20000, -300), (220000, 120000, -1700), (120000, 220000, -700)],
    # some example in tenths of mm
    [(200, 200, -300), (2200, 1200, -1700), (1200, 2200, -700)],
    [(20000, 20000, -300), (220000, 120000, -1700), (120000, 220000, -700)],
    [(200, 200, -300), (2200, 1200, -1700), (1200, 2200, -700)],
]

for points in samples:
    verify_plane(points)

    print("====[Translated]=========")
    # Translate plane to origin at P (simplifies by removing K coefficient)
    # A*x' + B*y' + C*z' = 0
    P = points[0]
    T = translate((0, 0, 0), P)
    xpoints = [translate(p, P) for p in points]
    verify_plane(xpoints)
    print("=========================\n")
