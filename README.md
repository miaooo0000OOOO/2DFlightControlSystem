# 二维飞控系统模拟

## 误差随时间变化绘图

![error-time](https://github.com/miaooo0000OOOO/2DFlightControlSystem/output/error-time.png)

## 物理模型

刚体物理引擎模拟

原理是 Sum(F) = ma

Sum(M) = J*omiga'

其中Sum()为求和符号, F为力, m为质量, a为加速度

M为力矩, J为转动惯量, omiga'为角加速度

使用半隐式物理引擎

## 控制算法

目标Sx与传感器Sx作差 -> Sx控制器 -> 输出目标角度与传感器角度作差 ->角度控制器 -> 升力差的一半

目标H与传感器H作差 -> H控制器 -> 升力均值

F1, F2 = 升力均值 +- 升力差的一半

其中Sx是无人机中心的x坐标, H是无人机中心的y坐标, 角度由x轴正半轴指向无人机正上方

因为升力有最大值, 所以升力均值不应超过最大升力的80%

## 运行代码

运行main.py会在ouput文件夹下输出名叫output.mp4的视频文件

以及error-time-table.csv和state-time-table.csv

运行tablePlot.py将会绘制误差随时间变化的图像

可以在main.py中的Env.mainLoop中找到IF_RENDER, 它控制是否渲染视频

pidController.py是控制系统的核心代码

renderer.py是图形渲染模块, geometry.py是用来渲染的简单几何体模块

tools.py可以用来转换字符串或元组形式的颜色为Opencv可用的颜色

## 参考文章

四旋翼无人机建模及控制

https://zhuanlan.zhihu.com/p/455984100