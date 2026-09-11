function [sys,x0,str,ts] = model_test_v2(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 9;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
function sys=mdlOutputs(t,x,u)
%% 定义参数
m1 = 2.0972;
m2 = 2.7077;
lm2 = 0.2719;
m3 = 3.0144;
l0 = 0.139;
l1 = 0.0845;
l2 = 0.4131;
l3 = 0.3702;
lm3 = 0.1876;
m1x = 0;
m1y = -0.013697;
m1z = 0.077671;
theta2 = 0.0461;
theta3 = 0.104;
g = 9.81;

I1xx=0.0033364;
I1xy=0;
I1xz=0;
I1yy=0.0026844;
I1yz=0.00011538;
I1zz=0.0025812;

I2xx=0.0029956;
I2xy=0.0012416;
I2xz=-0.00039946;
I2yy=0.0163;
I2yz=-0.0001901;
I2zz=0.016422;

I3xx=0.0029059;
I3xy=-0.00021301;
I3xz=-0.00031948;
I3yy=0.0031557;
I3yz=-0.00012645;
I3zz=0.0029024;
%% 获取传感器数据
q1 = u(1);
q2 = u(2);
q3 = u(3);
q1d = u(4);
q2d = u(5);
q3d = u(6);
qd = [q1d; q2d; q3d];
q1dd = u(7);
q2dd = u(8);
q3dd = u(9);
qdd = [q1dd; q2dd; q3dd];
%% 计算力矩
m11 = (   I3xx/2 + I2yy + I3yy/2 + I1zz - (I3xx*cos(2*q2))/2 + (I3yy*cos(2*q2))/2 + I3xy*sin(2*q2) + (l2^2*m3)/2 + (lm2^2*m2)/2 + (lm3^2*m3)/2 + (l2^2*m3*cos(2*q2) )/2 + (lm2^2*m2*cos(2*theta2 - 2*q2))/2 + (lm3^2*m3*cos(2*q2 - 2*theta3 + 2*q3))/2 + l2*lm3*m3*cos(theta3 - q3) + l2*lm3*m3*cos(2*q2 - theta3 + q3)  );
m12 = (I2yz + I3xz*sin(q2) + I3yz*cos(q2));
m13 = (I3xz*sin(q2) + I3yz*cos(q2));
m21 = (I2yz + I3xz*sin(q2) + I3yz*cos(q2));
m22 = (m3*l2^2 + 2*m3*cos(theta3 - q3)*l2*lm3 + m2*lm2^2 + m3*lm3^2 + I2zz + I3zz);
m23 = (m3*lm3^2 + l2*m3*cos(theta3 - q3)*lm3 + I3zz);
m31 = (I3xz*sin(q2) + I3yz*cos(q2));
m32 = (m3*lm3^2 + l2*m3*cos(theta3 - q3)*lm3 + I3zz);
m33 = (m3*lm3^2 + I3zz);

g1 = 0;
g2 = - g*lm3*m3*cos(q2 - theta3 + q3) - g*lm2*m2*cos(theta2 - q2) - g*l2*m3*cos(q2);
g3 = - g*lm3*m3*cos(q2 - theta3 + q3);

c11 = ((m2*sin(2*theta2 - 2*q2)*lm2^2)/2 + I3xy*cos(2*q2) + (I3xx*sin(2*q2))/2 - (I3yy*sin(2*q2))/2 - (l2^2*m3*sin(2*q2))/2 - (lm3^2*m3*sin(2*q2 - 2*theta3 + 2*q3))/2 - l2*lm3*m3*sin(2*q2 - theta3 + q3))*q2d + ((l2*lm3*m3*sin(theta3 - q3))/2 - (lm3^2*m3*sin(2*q2 - 2*theta3 + 2*q3))/2 - (l2*lm3*m3*sin(2*q2 - theta3 + q3))/2)*q3d;
c12 = ((m2*sin(2*theta2 - 2*q2)*lm2^2)/2 - I3xy*(2*sin(q2)^2 - 1) + (I3xx*sin(2*q2))/2 - (I3yy*sin(2*q2))/2 - (l2^2*m3*sin(2*q2))/2 - (lm3^2*m3*sin(2*q2 - 2*theta3 + 2*q3))/2 - l2*lm3*m3*sin(2*q2 - theta3 + q3))*q1d + (- I3yz*sin(q2) - I3xz*(2*sin(q2/2)^2 - 1))*q2d + (- (I3yz*sin(q2))/2 - (I3xz*(2*sin(q2/2)^2 - 1))/2)*q3d;
c13 = ((l2*lm3*m3*sin(theta3 - q3))/2 - (lm3^2*m3*sin(2*q2 - 2*theta3 + 2*q3))/2 - (l2*lm3*m3*sin(2*q2 - theta3 + q3))/2)*q1d + ((I3xz*cos(q2))/2 - (I3yz*sin(q2))/2)*q2d;

c21 = ((m3*sin(2*q2)*l2^2)/2 + m3*sin(2*q2 - theta3 + q3)*l2*lm3 + (m3*sin(2*q2 - 2*theta3 + 2*q3)*lm3^2)/2 + I3xy*(2*sin(q2)^2 - 1) - (I3xx*sin(2*q2))/2 + (I3yy*sin(2*q2))/2 - (lm2^2*m2*sin(2*theta2 - 2*q2))/2)*q1d + ((I3yz*sin(q2))/2 + (I3xz*(2*sin(q2/2)^2 - 1))/2)*q3d;
c22 = l2*lm3*m3*sin(theta3 - q3)*q3d;
c23 = ((I3yz*sin(q2))/2 - (I3xz*cos(q2))/2)*q1d + l2*lm3*m3*sin(theta3 - q3)*q2d + l2*lm3*m3*sin(theta3 - q3)*q3d;

c31 = ((lm3^2*m3*sin(2*q2 - 2*theta3 + 2*q3))/2 - (l2*lm3*m3*sin(theta3 - q3))/2 + (l2*lm3*m3*sin(2*q2 - theta3 + q3))/2)*q1d + ((I3xz*cos(q2))/2 - (I3yz*sin(q2))/2)*q2d;
c32 = ((I3xz*cos(q2))/2 - (I3yz*sin(q2))/2)*q1d + (-l2*lm3*m3*sin(theta3 - q3))*q2d;
c33 = 0;

M = [m11 m12 m13;
    m21 m22 m23;
    m31 m32 m33];
C = [c11 c12 c13;
    c21 c22 c23;
    c31 c32 c33];
G = [g1; g2; g3];

T = M*qdd + C*qd + G;
% T1 = (I3xx/2 + I2yy + I3yy/2 + I1zz - (I3xx*cos(2*q2))/2 + (I3yy*cos(2*q2))/2 + I3xy*sin(2*q2) + (l2^2*m3)/2 + (lm2^2*m2)/2 + (lm3^2*m3)/2 + (l2^2*m3*cos(2*q2))/2 + (lm2^2*m2*cos(2*theta2 - 2*q2))/2 + (lm3^2*m3*cos(2*q2 - 2*theta3 + 2*q3))/2 + l2*lm3*m3*cos(theta3 - q3) + l2*lm3*m3*cos(2*q2 - theta3 + q3))*q1dd + (I2yz + I3xz*sin(q2) + I3yz*cos(q2))*q2dd + (I3xz*sin(q2) + I3yz*cos(q2))*q3dd + (- m3*sin(2*q2)*l2^2 - 2*m3*sin(2*q2 - theta3 + q3)*l2*lm3 + m2*sin(2*theta2 - 2*q2)*lm2^2 - m3*sin(2*q2 - 2*theta3 + 2*q3)*lm3^2 + 2*I3xy*cos(2*q2) + I3xx*sin(2*q2) - I3yy*sin(2*q2))*q1d*q2d + (l2*lm3*m3*sin(theta3 - q3) - lm3^2*m3*sin(2*q2 - 2*theta3 + 2*q3) - l2*lm3*m3*sin(2*q2 - theta3 + q3))*q1d*q3d + (I3xz*cos(q2) - I3yz*sin(q2))*q2d^2 + (I3xz*cos(q2) - I3yz*sin(q2))*q2d*q3d;
% T2 = (I2yz + I3xz*sin(q2) + I3yz*cos(q2))*q1dd + (m3*l2^2 + 2*m3*cos(theta3 - q3)*l2*lm3 + m2*lm2^2 + m3*lm3^2 + I2zz + I3zz)*q2dd + (m3*lm3^2 + l2*m3*cos(theta3 - q3)*lm3 + I3zz)*q3dd + ((m3*sin(2*q2)*l2^2)/2 + m3*sin(2*q2 - theta3 + q3)*l2*lm3 + (m3*sin(2*q2 - 2*theta3 + 2*q3)*lm3^2)/2 - I3xy*cos(2*q2) - (I3xx*sin(2*q2))/2 + (I3yy*sin(2*q2))/2 - (lm2^2*m2*sin(2*theta2 - 2*q2))/2)*q1d^2 + (I3yz*sin(q2) - I3xz*cos(q2))*q1d*q3d + 2*l2*lm3*m3*sin(theta3 - q3)*q2d*q3d + l2*lm3*m3*sin(theta3 - q3)*q3d^2 - g*lm3*m3*cos(q2 - theta3 + q3) - g*lm2*m2*cos(theta2 - q2) - g*l2*m3*cos(q2);
% T3 = (I3xz*sin(q2) + I3yz*cos(q2))*q1dd + (m3*lm3^2 + l2*m3*cos(theta3 - q3)*lm3 + I3zz)*q2dd + (m3*lm3^2 + I3zz)*q3dd + ((lm3^2*m3*sin(2*q2 - 2*theta3 + 2*q3))/2 - (l2*lm3*m3*sin(theta3 - q3))/2 + (l2*lm3*m3*sin(2*q2 - theta3 + q3))/2)*q1d^2 + (I3xz*cos(q2) - I3yz*sin(q2))*q1d*q2d + (-l2*lm3*m3*sin(theta3 - q3))*q2d^2 - g*lm3*m3*cos(q2 - theta3 + q3);



%% 输出
sys(1) = T(1);
sys(2) = T(2);
sys(3) = T(3);
