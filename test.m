% 定义系统参数
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.15;
g = 9.8;
l = 0.75;

p = I*(M+m) + M*m*l^2; % 矩阵 A 和 B 的分母

A = [0      1              0           0;
     0 -(I + m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [0;
     (I + m*l^2)/p;
     0;
     m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A, B, C, D, 'statename', states, 'inputname', inputs, 'outputname', outputs);

% 计算极点
poles = eig(A)

%计算可控性
co = ctrb(sys_ss);
controllability = rank(co)

% 定义 LQR 权重矩阵
Q = C'*C;
Q(1,1) = 100;  % 增加位置的权重
Q(3,3) = 100;   % 增加角度的权重
R = 1;
K = lqr(A, B, Q, R)

Ac = (A - B*K);
Bc = B;
Cc = C;
Dc = D;

sys_cl = ss(Ac, Bc, Cc, Dc, 'statename', states, 'inputname', inputs, 'outputname', outputs);

% 仿真参数
t = 0:0.01:5;  % 仿真时间
r = 0.2 * ones(size(t));  % 参考输入

% 设置初始状态：自然垂下，速度为 0
x0 = [25; 0; pi; 0];  % [x; dx; theta; dtheta]

% 进行仿真
[y, t, x] = lsim(sys_cl, r, t, x0);

% 计算作用力 u
u = -K * x';

% 绘制结果
figure;
[AX, H1, H2] = plotyy(t, y(:, 1), t, y(:, 2), 'plot');
set(get(AX(1), 'Ylabel'), 'String', 'cart position (m)')
set(get(AX(2), 'Ylabel'), 'String', 'pendulum angle (radians)')
title('Step Response with LQR Control')

% 绘制作用力
%figure;
%plot(t, u)
%xlabel('Time (s)')
%ylabel('Control Force (N)')
%title('Control Force over Time')

%figure;
%plot(t, x(:, 4))  % x的第四列是角速度phi_dot
%xlabel('Time (s)')
%ylabel('Angular Velocity (rad/s)')
%title('Pendulum Angular Velocity over Time')

%figure;
%plot(t, x(:, 2))  % x的第四列是角速度phi_dot
%xlabel('Time (s)')
%ylabel('sudu')
%title('sudu')
