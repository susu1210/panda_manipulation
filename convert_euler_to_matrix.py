from math import *
def convertEuler2Matrix(euler,order):

    a=euler[0]
    b=euler[1]
    c=euler[2]
    if order=='XZX':
        R=[[cos(b),-cos(c)*sin(b),sin(b)*sin(c)],
           [cos(a)*sin(b),cos(a)*cos(b)*cos(c)-sin(a)*sin(c),-cos(c)*sin(a)-cos(a)*cos(b)*sin(c)],
           [sin(a)*sin(b),cos(a)*sin(c)+cos(b)*cos(c)*sin(a),cos(a)*cos(c)-cos(b)*sin(a)*sin(c)]]
    elif order=='XYX':
        R=[[cos(b),sin(b)*sin(c),cos(c)*sin(b)],
           [sin(a)*sin(b),cos(a)*cos(c)-cos(b)*sin(a)*sin(c),-cos(a)*sin(c)-cos(b)*cos(c)*sin(a)],
           [-cos(a)*sin(b),cos(c)*sin(a)+cos(a)*cos(b)*sin(c),cos(a)*cos(b)*cos(c)-sin(a)*sin(c)]]
    elif order=='YXY':
        R=[[cos(a)*cos(c)-cos(b)*sin(a)*sin(c),sin(a)*sin(b),cos(a)*sin(c)+cos(b)*cos(c)*sin(a)],
           [sin(b)*sin(c),cos(b),-cos(c)*sin(b)],
           [-cos(c)*sin(a)-cos(a)*cos(b)*sin(c),cos(a)*sin(b),cos(a)*cos(b)*cos(c)-sin(a)*sin(c)]]
    elif order=='YZY':
        R=[[cos(a)*cos(b)*cos(c)-sin(a)*sin(c),-cos(a)*sin(b),cos(c)*sin(a)+cos(a)*cos(b)*sin(c)],
           [cos(c)*sin(b),cos(b),sin(b)*sin(c)],
           [-cos(a)*sin(c)-cos(b)*cos(c)*sin(a),sin(a)*sin(b),cos(a)*cos(c)-cos(b)*sin(a)*sin(c)]]
    elif order=='ZYZ':
        R=[[cos(a)*cos(b)*cos(c)-sin(a)*sin(c),-cos(c)*sin(a)-cos(a)*cos(b)*sin(c),cos(a)*sin(b)],
           [cos(a)*sin(c)+cos(b)*cos(c)*sin(a),cos(a)*cos(c)-cos(b)*sin(a)*sin(c),sin(a)*sin(b)],
           [-cos(c)*sin(b),sin(b)*sin(c),cos(b)]]
    elif order=='ZXZ':
        R=[[cos(a)*cos(c)-cos(b)*sin(a)*sin(c),-cos(a)*sin(c)-cos(b)*cos(c)*sin(a),sin(a)*sin(b)],
           [cos(c)*sin(a)+cos(a)*cos(b)*sin(c),cos(a)*cos(b)*cos(c)-sin(a)*sin(c),-cos(a)*sin(b)],
           [sin(b)*sin(c),cos(c)*sin(b),cos(b)]]
    elif order=='XZY':
        R=[[cos(b)*cos(c),-sin(b),cos(b)*sin(c)],
           [sin(a)*sin(c)+cos(a)*cos(c)*sin(b),cos(a)*cos(b),cos(a)*sin(b)*sin(c)-cos(c)*sin(a)],
           [cos(c)*sin(a)*sin(b)-cos(a)*sin(c),cos(b)*sin(a),cos(a)*cos(c)+sin(a)*sin(b)*sin(c)]]
    elif order=='XYZ':
        R=[[cos(b)*cos(c),-cos(b)*sin(c),sin(b)],
           [cos(a)*sin(c)+cos(c)*sin(a)*sin(b),cos(a)*cos(c)-sin(a)*sin(b)*sin(c),-cos(b)*sin(a)],
           [sin(a)*sin(c)-cos(a)*cos(c)*sin(b),cos(c)*sin(a)+cos(a)*sin(b)*sin(c),cos(a)*cos(b)]]
    elif order=='YXZ':
        R=[[cos(a)*cos(c)+sin(a)*sin(b)*sin(c),cos(c)*sin(a)*sin(b)-cos(a)*sin(c),cos(b)*sin(a)],
           [cos(b)*sin(c),cos(b)*cos(c),-sin(b)],
           [cos(a)*sin(b)*sin(c)-cos(c)*sin(a),cos(a)*cos(c)*sin(b)+sin(a)*sin(c),cos(a)*cos(b)]]
    elif order=='YZX':
        R=[[cos(a)*cos(b),sin(a)*sin(c)-cos(a)*cos(c)*sin(b),cos(c)*sin(a)+cos(a)*sin(b)*sin(c)],
           [sin(b),cos(b)*cos(c),-cos(b)*sin(c)],
           [-cos(b)*sin(a),cos(a)*sin(c)+cos(c)*sin(a)*sin(b),cos(a)*cos(c)-sin(a)*sin(b)*sin(c)]]
    elif order=='ZYX':
        R=[[cos(a)*cos(b),cos(a)*sin(b)*sin(c)-cos(c)*sin(a),sin(a)*sin(c)+cos(a)*cos(c)*sin(b)],
           [cos(b)*sin(a),cos(a)*cos(c)+sin(a)*sin(b)*sin(c),cos(c)*sin(a)*sin(b)-cos(a)*sin(c)],
           [-sin(b),cos(b)*sin(c),cos(b)*cos(c)]]
    elif order=='ZXY':
        R=[[cos(a)*cos(c)-sin(a)*sin(b)*sin(c),-cos(b)*sin(a),cos(a)*sin(c)+cos(c)*sin(a)*sin(b)],
           [cos(c)*sin(a)+cos(a)*sin(b)*sin(c),cos(a)*cos(b),sin(a)*sin(c)-cos(a)*cos(c)*sin(b)],
           [-cos(b)*sin(c),sin(b)*cos(b)*cos(c)]]
    else:
        R=[]

    return R