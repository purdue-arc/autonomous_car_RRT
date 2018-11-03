close all

length = 0.4;
width = 0.2;
mass = 4;
vx = 2; %constant foward velocity  
initial_state = [1, 1, 0, 0, 0]'; %x,y center of gravity, theta, lateral speed(vy), yaw rate(r or thetadot)
cf = %front cornering stiffness coeff
cr = %rear cornering stiffness coeff
lf = length/2; %distance from center of gravity to front wheel
lr = length/2; %distance from center of gravity to rear wheel

k1 = stateDerivatives(initial_state, length, width, mass, vx, cf, cr, lf, lr, steeringAngle);


%initial_state = [1, 1, 0, 0, 0, 0, 0]'; %x,y center of gravity, theta, lateral speed, yaw rate
global_state = [1, 1, 0, 0, 0, 0, 0]'; %x y theta dx dy ax ay
dt = 0.01;
integral = zeros(2, 1);
