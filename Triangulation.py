import matplotlib.pyplot as plt
import csv
import math
import sys
import numpy as np

from shapely.geometry import Polygon
import matplotlib.pyplot as plt
from collections import namedtuple
from sympy import Triangle

plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True

Point = namedtuple('Point', ['x', 'y'])

def FormVector(StartpntOfVec,EndpntOfVec):
    Vector = np.asarray(EndpntOfVec) - np.asarray(StartpntOfVec)
    return Vector

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]))
    vector = vec / norm
    return vector

def convexity_check(x1, y1, x2, y2, x3, y3):
    next_adjVect = FormVector((x2, y2),(x3, y3))
    prev_adjVect = FormVector((x2, y2),(x1, y1))

    unit_vector_1 = Vec2UnitVec(next_adjVect)
    unit_vector_2 = Vec2UnitVec(prev_adjVect)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)

    return np.deg2rad(angle)

def CheckConvex_Vertex(P_Point, curr_point, N_point):
    convex_angle = convexity_check(P_Point.x, P_Point.y, curr_point.x, curr_point.y, N_point.x, N_point.y)
    if(convex_angle<180):
        diagonalcheck = P_Point.x * (N_point.y - curr_point.y) + curr_point.x * (P_Point.y - N_point.y) + N_point.x * (curr_point.y - P_Point.y)
        if(diagonalcheck < 0):
             return True
        else:
            return False
    else:
        return False
    

def VerifyEar_Conditions(p1, p2, p3, polygon):
    if(Check_point_inside_triangle(p1, p2, p3, polygon) and CheckConvex_Vertex(p1, p2, p3)  > 0):  
        return True
    else:
        return False

def Check_point_inside_triangle(p1, p2, p3, polygon):
    for pn in polygon:
        area = AreaOfTriangle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y)
        area1 = AreaOfTriangle(pn.x, pn.y, p2.x, p2.y, p3.x, p3.y)
        area2 = AreaOfTriangle(pn.x, pn.y, p1.x, p1.y, p3.x, p3.y)
        area3 = AreaOfTriangle(pn.x, pn.y, p1.x, p1.y, p2.x, p2.y)
        if pn in (p1, p2, p3):
            continue
        elif(area == sum([area1, area2, area3])):
            return False
    return True

def AreaOfTriangle(trivert_x1, trivert_y1, trivert_x2, trivert_y2, trivert_x3, trivert_y3):
    Tri_area = 0.5 * ((trivert_x1*(trivert_y2-trivert_y3)) + (trivert_x2*(trivert_y3-trivert_y1)) + (trivert_x3*(trivert_y1-trivert_y2)))
    return abs(Tri_area)
        
def Triangulate(polygon4tri):
    polygon4tri = [Point(*curr_vert) for curr_vert in polygon4tri]
    ears = []
    triangles = []
    ear_flag = False

    polyVertices = len(polygon4tri)
    for count in range(polyVertices):
        AdjVert_prev = polygon4tri[count - 1]
        curr_vert = polygon4tri[count]

        if(count+1 == polyVertices):
            adj_vert_index = 0
        else:
            adj_vert_index = (count + 1) 

        adj_vertex = polygon4tri[adj_vert_index]
        if VerifyEar_Conditions(AdjVert_prev, curr_vert, adj_vertex, polygon4tri):
            ears.append(curr_vert)
            ear_flag = True
    
    # Only if there is a ear we proceed further. polygons with 4 or more vertices have atleast two non-overlapping vertices

    if ear_flag == False:
        return -1

    while polyVertices >= 3:
        ear = ears.pop(0)
        count = polygon4tri.index(ear)

        if(count+1 == polyVertices):
            adj_vert_index = 0
        else:
            adj_vert_index = (count + 1) 

        adj_vertex = polygon4tri[adj_vert_index]
        AdjVert_prev = polygon4tri[count - 1]

        polygon4tri.remove(ear)
        polyVertices = polyVertices - 1 
        triangles.append(((AdjVert_prev.x, AdjVert_prev.y), (ear.x, ear.y), (adj_vertex.x, adj_vertex.y)))
        
        prev_AdjVert_prev = polygon4tri[count - 2]
        next_next_index = (count + 1) % polyVertices
        
        next_next_point = polygon4tri[next_next_index]

        consecutive_list = [(prev_AdjVert_prev, AdjVert_prev, adj_vertex, polygon4tri),(AdjVert_prev, adj_vertex, next_next_point, polygon4tri)]
        for list_point in consecutive_list:
            point = list_point[1]
            if VerifyEar_Conditions(*list_point):
                if point not in ears:
                    ears.append(point)
            elif point in ears:
                ears.remove(point)
    print(triangles)
    return triangles

CoOrd = []

with open('Input_37.txt') as f:
    reader = csv.reader(f)
    for count, (c1, c2) in enumerate(reader):
        CoOrd.append((float(c1),float(c2)))

for count in range(0,len(CoOrd)):
    print(count,CoOrd[count])

if(len(CoOrd)>=3):

    print("Triangulation possible")

    seperate_tri = Triangulate(CoOrd)

    print("total tringle",len(seperate_tri))
    # Visualising the polygon after triangulation
    for count in range(0,len(seperate_tri)):
        print(seperate_tri[count])
        current_tri = seperate_tri[count]
        for count_tri in range(0,len(current_tri)):
            print(current_tri[count_tri],count_tri)
            polygon1 = Polygon(current_tri)
            x, y = polygon1.exterior.xy
            plt.plot(x, y, c="blue")

    polygon1 = Polygon(CoOrd)
    x, y = polygon1.exterior.xy
    plt.plot(x, y, c="red")
    plt.show()

else:
    print("Triangulation not possible, due to less number of vertices")