import time
from geometry import *
import numpy as np
import cv2
    
def inputKeyIs(inputKey, value):
    if type(value)== type(""):
        return inputKey & 0xFF == ord(value)
    else:
        return inputKey & 0xFF == value

class Renderer:
    def __init__(self, cinemaPos=np.array([0,0]), zoomRatio=2, screenSize=960):
        self.objList = []
        self.nameList = []
        self.p = cinemaPos
        self.ratio = zoomRatio
        self.screen = screenSize
        self.img = np.zeros((self.screen, self.screen, 3), np.uint8)
        self.stepCount = 0
        self.customStateDict = {}

    def addObj(self, name, obj):
        assert name not in self.nameList and name != ''
        self.objList.append(obj)
        self.nameList.append(name)
    
    def getIndexByName(self, name):
        return self.nameList.index(name)
    
    def getObjByName(self, name):
        return self.objList[self.getIndexByName(name)]
    
    def delObj(self, name):
        self.objList.pop(self.getIndexByName(name))
        self.nameList.remove(name)

    def render(self):
        for index, obj in enumerate(self.objList):
            UIflag = self.nameList[index][0]=='@'
            if type(obj) == Rect:       # 矩形
                # 视图变换
                center = obj.center.copy()
                ab = obj.ab.copy()
                if UIflag:
                    center = center * self.screen/2 + self.screen/2
                    ab *= self.screen/2
                else:
                    center = center - self.p
                    center = center * self.screen/self.ratio + self.screen/2
                    ab *= self.screen/self.ratio

                # 渲染
                a, b = ab[0], ab[1]
                Mangle = np.array(
                    [[np.cos(obj.angle), -np.sin(obj.angle)],
                    [np.sin(obj.angle), np.cos(obj.angle)]]
                    )
                p1 = center + np.dot(Mangle, np.array([a, b]))
                p2 = center + np.dot(Mangle, np.array([-a, b]))
                p3 = center + np.dot(Mangle, np.array([-a, -b]))
                p4 = center + np.dot(Mangle, np.array([a, -b]))
                p1[1], p2[1], p3[1], p4[1] = self.screen-p1[1], self.screen-p2[1], self.screen-p3[1], self.screen-p4[1]
                pts = np.stack((p1, p2, p3, p4)).reshape((-1, 1, 2)).astype(np.int32)
                cv2.fillPoly(self.img, [pts], obj.color)
            elif type(obj) == Circle:           # 圆
                center = obj.center.copy()
                r = obj.r
                if UIflag:
                    center = center * self.screen/2 + self.screen/2
                    r *= self.screen/2
                else:
                    center = center - self.p
                    center = center * self.screen/self.ratio + self.screen/2
                    r *= self.screen/self.ratio
                center[1] = self.screen-center[1]
                center = center.astype(np.int32)
                cv2.circle(self.img, center, int(r), obj.color, -1)
            elif type(obj) == Arrow or type(obj) == Line:   # 直线和箭头
                start, end = obj.start.copy(), obj.end.copy()
                if UIflag:
                    start = start * self.screen/2 + self.screen/2
                    end = end * self.screen/2 + self.screen/2
                else:
                    start, end = start - self.p, end - self.p
                    start = start * self.screen/self.ratio + self.screen/2
                    end = end * self.screen/self.ratio + self.screen/2
                start[1], end[1] = self.screen-start[1], self.screen-end[1]
                start, end = start.astype(np.int32), end.astype(np.int32)
                if type(obj) == Arrow:
                    cv2.arrowedLine(self.img, start, end, obj.color, obj.thickness)
                else:
                    cv2.line(self.img, start, end, obj.color, obj.thickness)
            else:
                raise
        self.stepCount += 1
    
    def clear(self):
        self.img = np.zeros((self.screen, self.screen, 3), np.uint8)
    
    def window(self, callback=None, args=None, mouseCallback=None):
        # callback(self:Renderer, key:int, args:dict) -> None
        cv2.namedWindow("Renderer2D")
        if mouseCallback is not None:
            cv2.setMouseCallback("Renderer2D", mouseCallback)

        while True:
            FPS = 240
            DT = 1/FPS
            k = cv2.waitKey(int(DT*1000))
            if callback is not None:
                callback(self, k ,args)
            self.render()
            cv2.imshow("Renderer2D", self.img)
            up = np.array([0, 1])
            right = np.array([1, 0])
            speed = 10
            zoomSpeed = 20
            ds = np.array([0, 0])
            if inputKeyIs(k, 'w'):
                ds = DT * speed * up
            elif inputKeyIs(k, 's'):
                ds = -DT * speed * up
            if inputKeyIs(k, 'a'):
                ds = -DT * speed * right
            elif inputKeyIs(k, 'd'):
                ds = DT * speed * right
            if inputKeyIs(k, 'q'):
                self.ratio += zoomSpeed * DT
            elif inputKeyIs(k, 'e'):
                self.ratio -= zoomSpeed * DT
            if inputKeyIs(k, 27):
                break
            self.p = self.p + ds
            self.clear()
            time.sleep(DT)

if __name__ == "__main__":
    r = Renderer()
    r.addObj('droneBodyR', Rect(np.array([0,0]), 0.25, 0.1, 0, 'blue'))
    r.addObj('targetPointC', Circle(np.array([0.5,0.3]), 0.04, '#f0a020'))
    r.addObj('@bottomLine-', Line(np.array([-0.6,-0.8]), np.array([0.6,-0.8]), 2, 'red'))
    r.addObj('@bottomLeftLine|', Line(np.array([-0.6, -0.9]), np.array([-0.6,-0.7]), 4, 'red'))
    r.addObj('@bottomRightLine|', Line(np.array([0.6,-0.9]), np.array([0.6, -0.7]), 4, 'red'))

    r.addObj('@rightLine|', Line(np.array([0.8,0.6]), np.array([0.8, -0.6]), 2, 'red'))
    r.addObj('@rightTopLine-', Line(np.array([0.7,0.6]), np.array([0.9, 0.6]), 4, 'red'))
    r.addObj('@rightBottomLine-', Line(np.array([0.7,-0.6]), np.array([0.9, -0.6]), 4, 'red'))

    r.addObj('@c1', Circle(np.array([0.8, -0.8]), 0.15, 'green'))

    def callback(self:Renderer, k, args):
        if '@bottomLinePointer|' not in self.nameList:
            self.addObj('@bottomLinePointer|', Line(np.array([0,0]), np.array([0,0]), 3, 'red'))
        obj = self.getObjByName('@bottomLinePointer|')
        index = self.getIndexByName('@bottomLinePointer|')
        x = self.p[0]/2
        self.objList[index] = Line(np.array([x, -0.85]), np.array([x, -0.75]), obj.thickness, obj.color)
        if x < -0.6 or x > 0.6:
            self.delObj('@bottomLinePointer|')

        if '@RightLinePointer-' not in self.nameList:
            self.addObj('@RightLinePointer-', Line(np.array([0,0]), np.array([0,0]), 3, 'red'))
        obj = self.getObjByName('@RightLinePointer-')
        index = self.getIndexByName('@RightLinePointer-')
        y = self.p[1]/2
        self.objList[index] = Line(np.array([0.85, y]), np.array([0.75, y]), obj.thickness, obj.color)
        if y < -0.6 or y > 0.6:
            self.delObj('@RightLinePointer-')

    r.window(callback)

# renderer = Renderer(np.array([0, 0]), 2, 960)
# renderer.addObj('r1', Rect(np.array([0,0]), 0.1, 0.3, np.pi/4, (0,0,255)))
# renderer.addObj('@c1', Circle(np.array([0,1]), 0.1, (0,255,0)))       # @ 开头表示其为UI界面，不随摄像机移动而变化
# # renderer.addObj('@a1', ArrowPol(np.array([0,1]), np.pi/2, 3, 1, (255,0,0)))
# renderer.addObj('a2', Arrow(np.array([-0.8,-0.8]), np.array([0,-1]), 3, (0,0,255)))
# renderer.addObj('l1', Line(np.array([-0.8,-0.8]), np.array([0,1]), 3, (0,0,255)))
# renderer.render()

# # def callback(self:Renderer, k, args):
# #     if 'colorFlag' not in self.customStateDict:
# #         self.customStateDict['colorFlag'] = True
# #     obj:Circle = self.getObjByName('@c1')
# #     if inputKeyIs(k, 'p'):
# #         self.customStateDict['colorFlag'] = not self.customStateDict['colorFlag']
# #     obj.color = (0,0,255) if self.customStateDict['colorFlag'] else (0,255,0)

# renderer.window(callback)
        