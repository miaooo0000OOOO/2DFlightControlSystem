import random as R
import time
from renderer import *
from pidController import FlightCtrl
import cv2
import numpy as np

g = 9.8
video = cv2.VideoWriter('output/output.mp4', cv2.VideoWriter_fourcc(*"MP4V"), 30, (960,960))

class Drone:
    def __init__(self, s, v, angle, angVel, shape:Rect, m:float):
        self.s = s      # [sx, sy] 位移
        self.v = v      # [vx, vy] 速度
        self.normal = np.array([np.cos(angle), np.sin(angle)])  # 方向向量 模长=1
        self.angle = angle
        self.angVel = angVel
        self.shape = shape
        self.m = m
        self.I = 1/3*m*(shape.a**2+shape.b**2)      # 转动惯量
        self.Fmax = 10      # 单桨叶能产生的最大升力
        self.Fmin = -10     # 单桨叶能产生的最大反向推力
        self.FlightContrller = FlightCtrl()

    def autoControl(self, target, dt):
        state = (self.s[0], self.s[1], self.angle)
        return self.FlightContrller(target, state, dt)

    def control(self, F1, F2):
        F1 = max(min(F1, self.Fmax), self.Fmin) # 左升力大小
        F2 = max(min(F2, self.Fmax), self.Fmin) # 右升力大小
        return np.array([F1, F2])

    def calculateF(self, Fs):
        lenF1, lenF2 = self.control(*Fs)
        F1 = lenF1 * self.normal
        F2 = lenF2 * self.normal
        return F1, F2

class Env:
    def __init__(self, drone:Drone, renderer:Renderer):
        self.drone = drone
        self.renderer = renderer
        self.dt = 1e-3
        self.t = 0
        self.stepCounter = 0
        self.logTable = np.array([])      # 数据记录
        self.errorLogTable = np.array([]) # 被控误差记录
        self.initRenderer()
        self.F_R = np.array([0,0])
    
    def initRenderer(self):
        r = self.renderer
        r.addObj('droneBodyR', Rect(np.array([0,0]), 0.25, 0.1, 0, 'blue'))
        r.addObj('@bottomLine-', Line(np.array([-0.6,-0.8]), np.array([0.6,-0.8]), 2, 'red'))
        r.addObj('@bottomLeftLine|', Line(np.array([-0.6, -0.9]), np.array([-0.6,-0.7]), 4, 'red'))
        r.addObj('@bottomRightLine|', Line(np.array([0.6,-0.9]), np.array([0.6, -0.7]), 4, 'red'))

        r.addObj('@rightLine|', Line(np.array([0.8,0.6]), np.array([0.8, -0.6]), 2, 'red'))
        r.addObj('@rightTopLine-', Line(np.array([0.7,0.6]), np.array([0.9, 0.6]), 4, 'red'))
        r.addObj('@rightBottomLine-', Line(np.array([0.7,-0.6]), np.array([0.9, -0.6]), 4, 'red'))

        r.addObj('@c1', Circle(np.array([0.8, -0.8]), 0.15, 'green'))
    
    def setTarget(self, targetPoints):
        self.targetPoints = targetPoints
    
    def step(self):
        d = self.drone
        timeRangeLeft = 0
        for item in self.targetPoints:
            if timeRangeLeft <= self.t <= item['t']:
                self.target = item['pos']
                break
            else:
                timeRangeLeft = item['t']
        self.F1, self.F2 = d.calculateF(d.autoControl(self.target, self.dt))  # 左右两边升力的大小

        # 角度计算
        M90 = np.array([[0, -1], [1, 0]])   # 使向量逆时针转90度
        u = np.dot(M90, d.normal)       # 由矩形中心指向左边的单位向量
        r = d.shape.a / 2               # 力臂 = 矩形长的一半
        MF1, MF2 = np.cross(self.F1, -r*u), np.cross(self.F2, r*u)    # 力矩
        angAcc = (MF1 + MF2) / d.I    # 角加速度
        self.drone.angVel += angAcc * self.dt
        self.drone.angle += d.angVel * self.dt
        self.drone.angle = d.angle % (2 * np.pi)
        self.drone.normal = np.array([np.cos(self.drone.angle), np.sin(self.drone.angle)])

        # 位移计算
        Fsum = self.F1+self.F2                    # 升力
        self.G = np.array([0, -d.m*g])
        Fsum += self.G   # 重力
        LimRandomF = 0   # 随机扰动的最大值
        Ra = R.random()*2*np.pi # 随机角度
        Rl = R.random()*LimRandomF/10 # 随机大小 # 000000000000000
        self.F_R = self.F_R + 0.025*np.array([np.cos(Ra), np.sin(Ra)])
        if np.linalg.norm(self.F_R) > LimRandomF:
            self.F_R = self.F_R/np.linalg.norm(self.F_R)*LimRandomF
        Fsum += self.F_R        # 随机扰动
        a = Fsum / d.m
        self.drone.v += a * self.dt
        self.drone.s += self.drone.v * self.dt
        
        # 计数器自增
        self.stepCounter += 1
        self.t += self.dt

    def logData(self):
        d = self.drone
        # t, sx, sy, vx, vy, angle, angVel
        self.logTable = np.append(self.logTable, [self.t, d.s[0], d.s[1], d.v[0], d.v[1], d.angle, d.angVel]).reshape((-1, 7))
        # t, sx, angle, h
        fc = d.FlightContrller
        self.errorLogTable = np.append(self.errorLogTable,
        [self.t, fc.SxCtrl.pre_error, fc.angleCtrl.pre_error, fc.HCtrl.pre_error]).reshape((-1, 4))
    
    def __str__(self):
        d = self.drone
        return "t:%.3f\ts:%s\tv:%s\tangle:%s度\tangVel:%s度/s\n角度差:%.3f度" % \
        (self.t, str(d.s), str(d.v), str(d.angle/np.pi*180), str(d.angVel/np.pi*180))

    def mainLoop(self, callback):
        # cv2.namedWindow("Renderer2D")
        FPS = 30
        DT = 1/FPS
        while self.t < 20:
            self.step()
            if self.stepCounter % int(DT/self.dt) == 0:
                IF_RENDER = True
                if IF_RENDER:
                    cv2.waitKey(int(DT*1000)) 
                    callback(self)
                    self.renderer.render() 
                    video.write(self.renderer.img)
                    cv2.imshow("Renderer2D", self.renderer.img)
                    time.sleep(DT)
                    self.renderer.clear()
                self.logData()
                # print(self)
        print('loopEnd')


fl = np.float
d = Drone(s=np.array([0, 0], fl),
    v=np.array([0, 0], fl),
    angle=np.pi/2,
    angVel=0,
    shape=Rect(np.array([0,0]),0.5,0.2, 0, 'blue'),
    m=1)
r = Renderer(zoomRatio=15)
e = Env(d, r)
# targetPoints = [{'t':3,'pos':np.array([3,4])},
#     {'t':6,'pos':np.array([10, 20])}]
targetPoints = [{'t':3,'pos':np.array([10,10])}]
e.setTarget(targetPoints)

def callback(self):
    self.renderer.p = self.drone.s
    droneBodyR:Rect = self.renderer.getObjByName('droneBodyR')
    droneBodyR.angle = self.drone.angle - np.pi/2
    droneBodyR.center = self.drone.s

    rd = self.renderer
    xAxisRatio, yAxisRatio = 100, 100
    ForceZoom = 20
    # x轴
    if '@bottomLinePointer|' not in rd.nameList:
        rd.addObj('@bottomLinePointer|', Line(np.array([0,0]), np.array([0,0]), 3, 'red'))
    obj = rd.getObjByName('@bottomLinePointer|')
    index = rd.getIndexByName('@bottomLinePointer|')
    x = rd.p[0]/xAxisRatio
    rd.objList[index] = Line(np.array([x, -0.85]), np.array([x, -0.75]), obj.thickness, obj.color)
    if x < -0.6 or x > 0.6:
        rd.delObj('@bottomLinePointer|')
    # y轴
    if '@RightLinePointer-' not in rd.nameList:
        rd.addObj('@RightLinePointer-', Line(np.array([0,0]), np.array([0,0]), 3, 'red'))
    obj = rd.getObjByName('@RightLinePointer-')
    index = rd.getIndexByName('@RightLinePointer-')
    y = rd.p[1]/yAxisRatio
    rd.objList[index] = Line(np.array([0.85, y]), np.array([0.75, y]), obj.thickness, obj.color)
    if y < -0.6 or y > 0.6:
        rd.delObj('@RightLinePointer-')
    # F1
    if 'F1A' not in rd.nameList:
        rd.addObj('F1A', ArrowPol(np.array([0,0]), 0, 0, 2, 'green'))
    obj = rd.getObjByName('F1A')
    M90 = np.array([[0, -1], [1, 0]])
    obj.start = self.drone.s + self.drone.shape.a/2 * np.dot(M90, self.drone.normal)
    # obj.end = obj.start + np.linalg.norm(self.F1)/10*self.drone.normal
    obj.end = obj.start + self.F1/ForceZoom

    # F2
    if 'F2A' not in rd.nameList:
        rd.addObj('F2A', ArrowPol(np.array([0,0]), 0, 0, 2, 'green'))
    obj = rd.getObjByName('F2A')
    obj.start = self.drone.s - self.drone.shape.a/2 * np.dot(M90, self.drone.normal)
    # obj.end = obj.start + np.linalg.norm(self.F2)/10*self.drone.normal
    obj.end = obj.start + self.F2/ForceZoom
    
    # F_R
    if 'F_R' not in rd.nameList:
        rd.addObj('F_R', ArrowPol(np.array([0,0]), 0, 0, 2, 'green'))
    obj = rd.getObjByName('F_R')
    obj.start = self.drone.s
    obj.end = obj.start + self.F_R/ForceZoom

    # G
    if 'G' not in rd.nameList:
        rd.addObj('G', ArrowPol(np.array([0,0]), 0, 0, 2, 'green'))
    obj = rd.getObjByName('G')
    obj.start = self.drone.s
    obj.end = obj.start + self.G/ForceZoom

    # v
    if 'v' not in rd.nameList:
        rd.addObj('v', ArrowPol(np.array([0,0]), 0, 0, 2, 'yellow'))
    obj = rd.getObjByName('v')
    obj.start = self.drone.s
    obj.end = obj.start + self.drone.v/30

    # targetPoint
    if 'targetPoint' not in rd.nameList:
        rd.addObj('targetPoint', Circle(np.array([0,0]), 0.05, '#ff9900'))
    obj = rd.getObjByName('targetPoint')
    obj.center = self.target

    # radarA
    if '@radarA' not in rd.nameList:
        rd.addObj('@radarA', ArrowPol(np.array([0,0]), 0, 0, 5, '#dd22dd'))
    obj = rd.getObjByName('@radarA')
    obj.start = np.array([0.8, -0.8])
    error = self.target - self.drone.s
    if np.linalg.norm(error) != 0:
        error = error/np.linalg.norm(error)
    obj.end = obj.start + error*0.15

try:
    e.mainLoop(callback)
except KeyboardInterrupt:
    pass
finally:
    video.release()
    np.savetxt('output/state-t-table.csv', e.logTable, delimiter=',')
    np.savetxt('output/error-t-table.csv', e.errorLogTable, delimiter=',')
    print('end')