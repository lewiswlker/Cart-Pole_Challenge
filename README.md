# Cart-Pole Challenge简介
Cart-pole Challenge是一个由Matlab代码编写的仿真环境，本项目是一个完成作业。项目主要要求挑战者编写Matlab代码控制倒立摆至竖直向上的位置。控制量为作用在小车上的力，具体模型说明参考[链接](https://zhuanlan.zhihu.com/p/54071212),仿真器界面见下图
<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Cart-Pole/pics/cart-pole.png width=800%/>
</div>
上面的代码还缺少movie文件夹，可以参考<a href="https://gitee.com/coralab/ic-challenge/tree/master/Cart-Pole">Cart-Pole</a>，下载从而完成补充。


## 1. 倒立摆模型
倒立摆动力学模型具体描述见[链接](https://zhuanlan.zhihu.com/p/54071212)，或如下图所示
<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Cart-Pole/pics/cart-pole-model.jpeg width=30%/>
</div>
注：文中分析和仿真中的模型角度$\theta$都是定义为杆子与竖直向下方向的夹角

摆立摆的动力学模型是一个非线性模型，如下所示

$
\begin{array}{c}
(M+m)\ddot{x}+b\dot{x}+ml\ddot{\theta}\cos\theta-ml\dot{\theta}^{2}\sin\theta=F\\
(I+ml^{2})\ddot{\theta}+mgl\sin\theta=-ml\ddot{x}\cos\theta
\end{array}
$


其中，$M,m$分别是小车和杆子质量，$b$为地面摩擦力，$x$是小车位移，$\theta$为杆子与竖直向下方向的夹角，$F$作用在小车上水平方向的力，$I$为杆子转动惯量。

另外仿真环境对小车作用设置了“饱和”机制（如下图所示），即挑战者传递给仿真器的小车作用力，将会被限制在一定范围内。

<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Arena/pics/saturation.png width=30%/>
</div>


## 2. Observation（当前环境信息）

仿真环境每隔一段时间，将会把仿真环境的信息以Observation类的形式告知挑战者，它的成员变量包含  
```
agent		%当前倒立摆信息   
t            	%当前所用时间  
```

## 3. 得分（暂无）


## 4. 设计控制策略
挑战者需要设计并提交一个Policy类文件，主要完成action函数。action函数传入参数为observation，传出action。仿真器会在特点的时间间隔调用action，依据挑战者设计的策略得到action，即控制量[u,v]，从而控制小车。目前，默认的一个策略为LQR控制器
```
classdef Policy < handle
        function action=action(self,observation)
 		  	sys=observation.agent;
          u=-10*sin(observation.t);
        	action=[u];
        end
end
```

## 5. Main函数
以下是Main函数的基本代码，main读取系统配置文件对仿真环境进行配置，之后进入仿真循环。在循环中，仿真器对Water Tank Challenge进行物理仿真，计算出水箱状态，并且每一次调用挑战者设计控制策略，然后把控制策略作用于水箱的控制中。
```
env = Env('sys.ini');   %读取系统配置文件
policy=Policy();
if (env.succeed)
    observation = env.reset();
    while 1
        env.render();
        action=policy.action(observation); %调用挑战者的控制策略
        [observation,done,info]=env.step(action); %物理仿真
        disp(info);
        if(done)
            break;
        end
        wait(100);
    end
end
```

## 6. 仿真配置文件sys.ini
为了方便挑战者进行测试，挑战者可以通过仿真配置文件sys.ini，进行相应配置。例如配置水箱控制饱和范围，水箱目标液位，是否录制游戏运行过程等。具体见该文件。
```
[StartPos]
x   = 10       % 倒立摆开始的水平位置
theta   = 0     % 倒立摆开始的角度
usat=100              % 饱和范围
linear=0      % 是否采用简单的线性模型还是复杂的非线性模型
cart_weight= 0.5  % 小车质量
pole_weight=0.2   % 杆子质量
```



