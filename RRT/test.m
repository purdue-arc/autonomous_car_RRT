%this is a test file
close all

dt = 0.01;
length = 0.4;
width = 0.2;
mass = 4;
steeringAngle = 0;
inertia = 1;
vx = 2; %constant foward velocity  
initial_state = [1, 1, 0, 0, 0]; %x,y center of gravity, theta, lateral speed(vy), yaw rate(r or thetadot)
cf = 0.1; %front cornering stiffness coeff
cr = 0.1; %rear cornering stiffness coeff
lf = length/2; %distance from center of gravity to front wheel
lr = length/2; %distance from center of gravity to rear wheel

%Fourth order Runge-Kutta
k1 = stateDerivatives(initial_state, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);
k2 = stateDerivatives(initial_state + k1./2, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);
k3 = stateDerivatives(initial_state + k2./2, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);
k4 = stateDerivatives(initial_state + k3, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);

next_state = initial_state + dt/6*(k1 + 2*k2 + 2*k3 + k4);

%initial_state = [1, 1, 0, 0, 0, 0, 0]'; %x,y center of gravity, theta, lateral speed, yaw rate
%global_state = [1, 1, 0, 0, 0, 0, 0]'; %x y theta dx dy ax ay
%integral = zeros(2, 1);
