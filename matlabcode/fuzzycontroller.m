function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
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
sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 25;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = 0;
str = [];
ts  = [];

function sys=mdlDerivatives(t,x,u)
sys=0;

function sys=mdlOutputs(t,x,u)
x1 = u(2); % angle difference between truck and trailer
x2 = u(3); % angle of trailer
x3 = u(4); % vertical position of rear end of trailer
velocity = -1.0; % constant speed of backing up, m/s
length_trailer = 6.5; % length of trailer, m
x = [x1;x2;x3]; % state variable x1,x2,x3
f1 = 1.2 * [1.2837, -0.4139, 0.0201]; % feedback gain
f2 = 1.2 * [0.9773, -0.0709, 0.0005];

temp = x2 + velocity * t /(2*length_trailer) * x1; % x2(k) + v*t/(2*L)*x1(k)

% if x2(k) + v*t/(2*L)*x1(k) is about pi (rad) or -pi (rad)
if (temp >= -pi && temp <= 0)
    membership(1) = -temp / pi;
elseif (temp > 0 && temp <= pi)
    membership(1) = temp / pi;
else
    membership(1) = 0;
end

% if x2(k) + v*t/(2*L)*x1(k) is about 0 (rad)
if (temp >= -pi && temp <= 0)
    membership(2) = (temp + pi) / pi;
elseif (temp > 0 && temp <= pi)
    membership(2) = (pi - temp) / pi;
else
    membership(2) = 0;
end

% rule1: if x2(k) + v*t/(2*L)-x1(k) is about pi (rad) or -pi (rad), then u(k) = f2*x(k)
if (membership(1) > membership(2))
    flag1 = 0;
    control = f2 * x;
% rule2: if x2(k) + v*t/(2*L)-x1(k) is about 0 (rad), then u(k) = f1*x(k)
elseif (membership(1) < membership(2))
    flag1 = 1;
    control = f1 * x;
else
    flag1 = 2;
    control = 0;
end

sys(1)=control;
sys(2)=flag1;