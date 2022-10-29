class PID:
    def __init__(self, Kp, Ki, Kd):
        self.setParam(Kp, Ki, Kd)
        self.integral = 0
        self.pre_error = 0

    def __call__(self, error, dt):
        self.integral += error * dt
        derivative = (error-self.pre_error) / dt
        y = self.Kp * error + self.Ki*self.integral + self.Kd * derivative
        self.pre_error = error
        return y
    
    def setParam(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd

    def zeroIntegral(self):
        self.integral = 0

class FlightCtrl:
    def __init__(self):
        self.angleCtrl = PID(2,0,1.5) # yes
        self.HCtrl = PID(4,1,4)     # yes
        self.SxCtrl = PID(0.3,0.06,0.4) #
    
    def __call__(self, target, state, dt):
        tSx, tH = target
        Sx, H, angle = state
        PI = 3.14159265358
        E = 2.718
        tAngle = -self.SxCtrl(tSx - Sx, dt)+PI/2
        print(self.SxCtrl.pre_error)
        print('控制目标角度:%.3f'%(tAngle/PI*180))
        tAngle = max(PI/6, min(5*PI/6, tAngle)) # target angle in (30, 150) degree
        errorAngle = (tAngle - angle)%(2*PI)
        errorAngle = errorAngle if errorAngle < PI else errorAngle-2*PI
        rotationCmd = self.angleCtrl(errorAngle, dt)
        # print('旋转命令:%.3f'%(rotationCmd))
        ThrustCmd = self.HCtrl(tH - H, dt)
        # print('推力命令:%.3f'%(ThrustCmd))
        ThrustCmd = min(ThrustCmd, 8)
        F1, F2 = ThrustCmd - rotationCmd, ThrustCmd + rotationCmd
        return F1, F2