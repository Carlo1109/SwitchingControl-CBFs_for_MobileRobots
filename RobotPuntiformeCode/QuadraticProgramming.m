clear all
close all
clc

global A B K kp kd mu delta delta1 freq Ox Oy R PosO alpha
global Pd Xd Yd PdDot XdDot YdDot Pd2Dot s
syms Pd Xd Yd PdDot XdDot YdDot Pd2Dot s

%Circle Trajectory and robot start position 
Ox = 10;
Oy = 10;
freq = pi/2;

%Obstacles Coordinates
PosO = [15; 18];

%PD Controller
kp = 100; 
kd = 30; 

%CBF - Paper
alpha = 5;
mu = 0.05;
delta = 2;
delta1 = 5;

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
Pd = [Ox + a*sin(freq*s); Oy + b*cos(freq*s)];
PdDot = [(freq*a*cos(freq*s)); (-freq*b*sin(freq*s))];
Pd2Dot = [(-(power(freq,2))*a*sin(freq*s)); (-(power(freq,2))*b*cos(freq*s))];
%Pd = [R*sin(freq*s)+Ox; R*cos(freq*s)+Oy];
%PdDot = [freq*R*cos(freq*s); (-freq*R*sin(freq*s))];
%Pd2Dot = [(-(power(freq,2))*R*sin(freq*s)); (-(power(freq,2))*R*cos(freq*s))];
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
X = x(:,1);
Y = x(:,2);
plot(X,Y, 'b', 'LineWidth', 0.7)
axis equal
hold on
plot(PosO(1), PosO(2),'bo','LineWidth', 2);
hold on 
circ1 = nsidedpoly(1000, 'Center', PosO', 'Radius', delta);
circ2 = nsidedpoly(1000, 'Center', PosO', 'Radius', delta1);
%traj = nsidedpoly(1000, 'Center', x0(1:2), 'Radius', R);
%plot(traj,'FaceColor','none', 'LineStyle', '--', 'EdgeColor', 'r')

%Define the parameters of the ellipse or the circle
angle = 0;         % rotation angle (in degrees)
    
%Generate the ellipse or circle
tel = linspace(0, 2*pi, 100);  % parameter values for the ellipse
xel = Ox + a*cos(t)*cosd(angle) - b*sin(t)*sind(angle);
yel = Oy + a*cos(t)*sind(angle) + b*sin(t)*cosd(angle);
    
%Plot the ellipse or circle
plot(xel, yel, 'r');
axis equal;   % ensure that the aspect ratio is 1:1

plot(circ1, 'FaceColor', 'r')
plot(circ2, 'FaceColor', 'yellow')

title('Robot Start Position:[10 10]; Obstacle Position:[2 5])')
xlabel('x')
ylabel('y')


%Control
function [xDot, ux, uy] = systemControl(t,x)
    global A B Pd PdDot Pd2Dot s kp kd PosO mu delta1 alpha delta

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
    distResidua = posR - PosO ;
    I2 = eye(2);
    E = mu*distResidua';
    L = (2+alpha*mu)*(distResidua'*velR) + mu*(velR'*velR) + alpha*(distResidua'*distResidua) - 2*alpha*delta ;
    options = optimset('Display', 'off');
    u = quadprog(I2, -2*uNominal, -E,L,[],[],[],[],[], options);

    ux = u(1);
    uy = u(2);
    xDot = A*x + B*u;
end


















