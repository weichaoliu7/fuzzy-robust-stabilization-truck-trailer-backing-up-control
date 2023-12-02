function [sys,x0,str,ts] = s_function(t,x,u,flag)

switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1
    sys=mdlDerivatives(t,x,u);
case 3
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 5;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 25;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
% initial position(x0 x1 x2 x3 x4)
x0=[0 0 0 20 0]; % case 1
% x0=[pi 0 pi 20 0]; % case 2
% x0=[pi/4 -pi/4 pi/2 -10 0]; % case 3
% x0=[pi/4 -pi/2 3*pi/4 -10 0]; % case 4
str=[];
ts=[];

function sys=mdlDerivatives(t,x,u)
control=u(1); % truck steering angle
flag1=u(2);
velocity = -1.0; % constant speed of backing up, m/s
length_truck = 2.8; % length of truck, m
length_trailer = 6.5; % length of trailer, m
d = (10^-2)/pi;

% simplified truck-trailer model
% if x2(k) + v*t/(2*L)*x1(k) is about pi (rad) or -pi (rad)
if flag1 == 0
    x_derivative(2) = -(velocity / length_trailer) * x(2) + (velocity / length_truck) * control; % derivative of angle difference between truck and trailer, Eq. 34
    x_derivative(3) = (velocity / length_trailer) * x(2); % derivative of angle of trailer
    x_derivative(4) = (d * velocity^2 * t)/(2 * length_trailer) * x(2) + d * velocity * x(3); % derivative of vertical position of rear end of trailer
    x_derivative(1) = x_derivative(2) + x_derivative(3); % derivative of angle of truck, Eq. 19's difference equation
    x_derivative(5) = velocity * cos(x(2)) * cos(x(3) + (velocity * t/length_trailer * sin(x(2)))/2); % derivative of horizontal position of rear end of trailer, Eq. 22
% if x2(k) + v*t/(2*L)*x1(k) is about 0 (rad)
elseif flag1 == 1 
    x_derivative(2) = -(velocity / length_trailer) * x(2) + (velocity / length_truck) * control; % derivative of angle difference between truck and trailer, Eq. 34
    x_derivative(3) = (velocity / length_trailer) * x(2); % derivative of angle of trailer
    x_derivative(4) = (velocity^2 * t)/(2 * length_trailer) * x(2) + velocity * x(3); % derivative of vertical position of rear end of trailer
    x_derivative(1) = x_derivative(2) + x_derivative(3); % derivative of angle of truck, Eq. 19's difference equation
    x_derivative(5) = velocity * cos(x(2)) * cos(x(3) + (velocity * t/length_trailer * sin(x(2)))/2); % derivative of horizontal position of rear end of trailer, Eq. 22
% Ichihashi truck-trailer model
elseif flag1 == 2
    x_derivative(1) = velocity / length_truck * control; % derivative of angle of truck, Eq. 18
    x_derivative(3) = velocity /length_trailer * sin(x(2)); % derivative of angle of trailer, Eq. 20
    x_derivative(2) = x_derivative(1) - x_derivative(3); % derivative of angle difference between truck and trailer, Eq. 19's difference equation
    x_derivative(4) = velocity * cos(x(2)) * sin(x(3) + (velocity * t/length_trailer * sin(x(2)))/2); % derivative of vertical position of rear end of trailer, Eq. 21
    x_derivative(5) = velocity * cos(x(2)) * cos(x(3) + (velocity * t/length_trailer * sin(x(2)))/2); % derivative of horizontal position of rear end of trailer, Eq. 22
end

sys(1)=x_derivative(1);
sys(2)=x_derivative(2);
sys(3)=x_derivative(3);
sys(4)=x_derivative(4);
sys(5)=x_derivative(5);

function sys=mdlOutputs(t,x,u)
sys(1)=x(1); % angle of truck

% 90 deg and -90 deg correspond to two jackknife positions
if x(2) <= -pi/2
    x(2) = -pi/2;
elseif x(2) >= pi/2
     x(2) = pi/2;
end

sys(2)=x(2); % angle difference between truck and trailer
sys(3)=x(3); % angle of trailer
sys(4)=x(4); % vertical position of rear end of trailer
sys(5)=x(5); % horizontal position of rear end of trailer

x0 = x(1); % angle of truck
x1 = x(2); % angle difference between truck and trailer
x2 = x(3); % angle of trailer
x3 = x(4); % vertical position of rear end of trailer
x4 = x(5); % horizontal position of rear end of trailer

length_truck = 2.8; % length of truck, m
length_trailer = 6.5; % length of trailer, m
length_jackknife = 0.7; % length of jackknife, m
width_truck = 2; % width of truck, m
width_trailer = 2; % width of trailer, m

point1(1) = x4 - width_trailer/2 * sin(x2); % rear left corner of trailer
point1(2) = x3 + width_trailer/2 * cos(x2);

point2(1) = x4 + width_trailer/2 * sin(x2); % rear right corner of trailer
point2(2) = x3 - width_trailer/2 * cos(x2);

point4(1) = point1(1) + length_trailer * cos(x2); % left front corner of trailer
point4(2) = point1(2) + length_trailer * sin(x2);

point3(1) = point2(1) + length_trailer * cos(x2); % right front corner of trailer
point3(2) = point2(2) + length_trailer * sin(x2);

point5(1) = x4 + length_trailer * cos(x2); % midpoint of front side of trailer
point5(2) = x3 + length_trailer * sin(x2);

point6(1) = point5(1) + length_jackknife * cos(x0); % midpoint of front side of trailer
point6(2) = point5(2) + length_jackknife * sin(x0); 

point7(1) = point6(1) - width_truck/2 * sin(x0); % rear left corner of truck
point7(2) = point6(2) + width_truck/2 * cos(x0);

point8(1) = point6(1) + width_truck/2 * sin(x0); % rear right corner of truck
point8(2) = point6(2) - width_truck/2 * cos(x0);

point10(1) = point7(1) + length_truck * cos(x0); % left front corner of truck
point10(2) = point7(2) + length_truck * sin(x0);

point9(1) = point8(1) + length_truck * cos(x0); % right front corner of truck
point9(2) = point8(2) + length_truck * sin(x0);

points = [point1; point2; point3; point4; point5; point6; point7; point8; point9; point10];

for i = 1:10
    sys(6+2*(i-1)) = points(i, 1);
    sys(7+2*(i-1)) = points(i, 2);
end