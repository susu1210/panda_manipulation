from math import *
def convertEuler2Quaternion(euler,order):

    a=euler[0]
    b=euler[1]
    c=euler[2]
    if order=='XZX':
        q = [sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * sin(b / 2.) * sin(c / 2.) - sin(a / 2.) * sin(b / 2.) * cos(c / 2.),
             cos(a / 2.) * sin(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * cos(b / 2.) * sin(c / 2.)]
    elif order=='XYX':
        q = [sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * sin(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.),
             sin(a / 2.) * sin(b / 2.) * cos(c / 2.) - cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * cos(b / 2.) * sin(c / 2.)]
    elif order=='YXY':
        q = [cos(a / 2.) * sin(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.),
             sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * sin(b / 2.) * sin(c / 2.) - sin(a / 2.) * sin(b / 2.) * cos(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * cos(b / 2.) * sin(c / 2.)]
    elif order=='YZY':
        q = [sin(a / 2.) * sin(b / 2.) * cos(c / 2.) - cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * sin(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * cos(b / 2.) * sin(c / 2.)]
    elif order=='ZYZ':
        q = [cos(a / 2.) * sin(b / 2.) * sin(c / 2.) - sin(a / 2.) * sin(b / 2.) * cos(c / 2.),
             cos(a / 2.) * sin(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.),
             sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * cos(b / 2.) * sin(c / 2.)]
    elif order=='ZXZ':
        q = [cos(a / 2.) * sin(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.),
             sin(a / 2.) * sin(b / 2.) * cos(c / 2.) - cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * cos(b / 2.) * sin(c / 2.)]
    elif order=='XZY':
        q = [sin(a / 2.) * cos(b / 2.) * cos(c / 2.) - cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             -sin(a / 2.) * sin(b / 2.) * cos(c / 2.) + cos(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * sin(b / 2.) * cos(c / 2.) + sin(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.)]
    elif order=='XYZ':
        q = [sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             -sin(a / 2.) * cos(b / 2.) * sin(c / 2.) + cos(a / 2.) * sin(b / 2.) * cos(c / 2.),
             cos(a / 2.) * cos(b / 2.) * sin(c / 2.) + sin(a / 2.) * sin(b / 2.) * cos(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * sin(b / 2.) * sin(c / 2.)]
    elif order=='YXZ':
        q = [sin(a / 2.) * cos(b / 2.) * sin(c / 2.) + cos(a / 2.) * sin(b / 2.) * cos(c / 2.),
             sin(a / 2.) * cos(b / 2.) * cos(c / 2.) - cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * sin(c / 2.) - sin(a / 2.) * sin(b / 2.) * cos(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.)]
    elif order=='YZX':
        q = [sin(a / 2.) * sin(b / 2.) * cos(c / 2.) + cos(a / 2.) * cos(b / 2.) * sin(c / 2.),
             sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             cos(a / 2.) * sin(b / 2.) * cos(c / 2.) - sin(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * sin(b / 2.) * sin(c / 2.)]
    elif order=='ZYX':
        q = [cos(a / 2.) * cos(b / 2.) * sin(c / 2.) - sin(a / 2.) * sin(b / 2.) * cos(c / 2.),
             cos(a / 2.) * sin(b / 2.) * cos(c / 2.) + sin(a / 2.) * cos(b / 2.) * sin(c / 2.),
             sin(a / 2.) * cos(b / 2.) * cos(c / 2.) - cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) + sin(a / 2.) * sin(b / 2.) * sin(c / 2.)]
    elif order=='ZXY':
        q = [cos(a / 2.) * sin(b / 2.) * cos(c / 2.) - sin(a / 2.) * cos(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * sin(c / 2.) + sin(a / 2.) * sin(b / 2.) * cos(c / 2.),
             sin(a / 2.) * cos(b / 2.) * cos(c / 2.) + cos(a / 2.) * sin(b / 2.) * sin(c / 2.),
             cos(a / 2.) * cos(b / 2.) * cos(c / 2.) - sin(a / 2.) * sin(b / 2.) * sin(c / 2.)]
    else:
        q=[]

    return q