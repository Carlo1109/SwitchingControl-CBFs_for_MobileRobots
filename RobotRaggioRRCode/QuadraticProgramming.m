clear all
close all
clc

global A B K kp kd mu delta delta1 freq Ox Oy R PosO alpha RR
global Pd Xd Yd PdDot XdDot YdDot Pd2Dot s
syms Pd Xd Yd PdDot XdDot YdDot Pd2Dot s

%Circle Trajectory and robot start position
Ox = 0;
Oy = 0;
freq = pi/2;
RR = 2;

%Obstacles Coordinates
PosO = [21; 9];

%PD Controller
kp = 100; 
kd = 30; 

%CBF - Paper
alpha = 5;
mu = 0.05;
delta = 2;
delta1 = 3;

%Frequency
freq = pi/2;

%System and Gain Matrix
A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];
K = [kp 0 kd 0; 0 kp 0 kd];

%Desidered Trajectory, Reference
R = 10;
tt = input('Inserire 0 per riferimento circolare e 1 per riferimento ellittico\n');
if tt == 0
    a = R;
    b = R;
    name = 'Safe circular trajectory tracking for mobile robots - QuadraticProgramming';
else
    a = 20;
    b = 10;
    name = 'Safe elliptical trajectory tracking for mobile robots - QuadraticProgramming';
end
Pd = [10 + a*sin(freq*s); 10 + b*cos(freq*s)];
PdDot = [(freq*a*cos(freq*s)); (-freq*b*sin(freq*s))];
Pd2Dot = [(-(power(freq,2))*a*sin(freq*s)); (-(power(freq,2))*b*cos(freq*s))];
Xd = Pd(1,:);
Yd = Pd(1,:);
XdDot = PdDot(1,:);
YdDot = PdDot(1,:);

%Feedback
tspan = 0 : 0.05 : 10;
x0 = [Ox Oy 0 0];
[t x] = ode45(@systemControl, tspan, x0);

posR = x(1:2,:);

%Plot
figure('Name', name)

%Trajectory of the robot and related surface lines
X = x(:,1);
Y = x(:,2);
XDot = x(:,3);
YDot = x(:,4);
dim = length(X);
x1 = zeros(dim,1);
y1 = zeros(dim,1);
x2 = zeros(dim,1);
y2 = zeros(dim,1);
for i=1:dim
    d = sqrt(XDot(i)^2+YDot(i)^2);
    sin_a = YDot(i)/d;
    cos_a = XDot(i)/d;
    
    x1(i) = X(i)-RR*sin_a;
    y1(i) = Y(i)+RR*cos_a;

    x2(i) = X(i)+RR*sin_a;
    y2(i) = Y(i)-RR*cos_a;
end

X1 = x1;
Y1 = y1;
X2 = x2;
Y2 = y2;
plot(X,Y, 'b', 'LineWidth', 0.7, 'DisplayName', 'Robot Trajectory')
hold on
plot(X1, Y1, 'y', 'LineWidth', 0.7, 'LineStyle','--', 'DisplayName', 'Robot Top Edge')
plot(X2, Y2, 'g', 'LineWidth', 0.7, 'LineStyle','--', 'DisplayName', 'Robot Bottom Edge')
axis equal

%Obstacles
hold on
plot(PosO(1), PosO(2), 'o', 'LineWidth', 2, 'HandleVisibility','off');
hold on 
circ1 = nsidedpoly(1000, 'Center', PosO', 'Radius', delta);
circ2 = nsidedpoly(1000, 'Center', PosO', 'Radius', delta1);
plot(circ1, 'FaceColor', 'r', 'HandleVisibility','off')
plot(circ2, 'FaceColor', 'r', 'HandleVisibility','off')

%Define the parameters of the ellipse or the circle
angle = 0;         % rotation angle (in degrees)
    
%Generate the ellipse or circle
tel = linspace(0, 2*pi, 100);
xel = 10 + a*cos(t)*cosd(angle) - b*sin(t)*sind(angle);
yel = 10 + a*cos(t)*sind(angle) + b*sin(t)*cosd(angle);
    
%Plot the ellipse or circle
plot(xel, yel, 'r', 'LineWidth', 0.8, 'LineStyle', '--', 'DisplayName','Reference', 'HandleVisibility','on');
axis equal;  

title(['Robot Start Position:[0 0]; Obstacle Position:[21 9])'])
xlabel('x')
ylabel('y')
legend('show', 'Location', 'best')


%Control
function [xDot, ux, uy] = systemControl(t,x)
    global A B Pd PdDot Pd2Dot s kp kd PosO mu alpha delta RR

    %Feedforward
    yRef = subs(Pd,s,t); 
    yRefDot =subs(PdDot,s,t);
    yRef2Dot = subs(Pd2Dot,s,t);
    yRef = double(yRef);
    yRefDot = double(yRefDot);
    yRef2Dot = double(yRef2Dot);
    
    %Position & Velocity
    posR = x(1:2);
    velR = x(3:4);

    %Nominal control input in Dtrack
    uNominal = yRef2Dot + kd*(yRefDot - velR) + kp*(yRef - posR);
    t

    %Quadratic Programming using CBF
    distResidua = (posR - PosO);
    I2 = eye(2);
    E = mu*distResidua';
    L = (2+alpha*mu)*(distResidua'*velR) + mu*(velR'*velR) + alpha*(distResidua'*distResidua-RR^2) - 2*alpha*(2*(delta+RR));
    options = optimset('Display', 'off');
    u = quadprog(I2, -2*uNominal, -E,L,[],[],[],[],[], options);

    ux = u(1);
    uy = u(2);
    xDot = A*x + B*u;
end


















