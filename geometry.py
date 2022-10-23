import numpy as np
import cv2
from tools import colorStr2BGR


class Rect:      # 矩形
    def __init__(self, center, a:float, b:float,angle:float, color):
        self.a = a      # 长
        self.b = b      # 宽
        self.angle = angle
        self.center = center
        self.color = colorStr2BGR(color)
        self.ab = np.array([a, b])
        self.S = a*b
    
    def __getitem__(self, index):
        assert index in [0, 1, slice(None, None, None)]
        if index == slice(None, None, None):
            return self.ab
        elif index == 0:
            return self.a
        else:
            return self.b
        
class Circle:
    def __init__(self, center:np.array, radius:float, color):
        self.center = center
        self.r = radius
        self.S = np.pi * self.r ** 2
        self.color = colorStr2BGR(color)
    
class Arrow:
    def __init__(self, start, end, thickness, color):
        self.start = start
        self.end = end
        self.thickness = thickness
        self.color = colorStr2BGR(color)

    def rotate(self, angle):
        v = self.end - self.start
        Mangle = np.array(
            [[np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]]
            )
        v_ = np.dot(Mangle, v)
        self.end = self.start + v_
    
    def zoom(self, scale):
        v = self.end - self.start
        v_ = scale * v
        self.end = self.start + v_

class Line(Arrow):
    pass


def ArrowPol(start, angle, length, thickness, color):
    v = np.array([length*np.cos(angle), length*np.sin(angle)])
    end = start + v
    return Arrow(start, end, thickness, color)

def LinePol(start, angle, length, thickness, color):
    v = np.array([length*np.cos(angle), length*np.sin(angle)])
    end = start + v
    return Line(start, end, thickness, color)