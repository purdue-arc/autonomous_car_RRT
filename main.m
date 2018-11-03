length = 0.4;
width = 0.2;
mass = 4;
speed =  
initial_state = [1, 1, 0, 0, 0, 0, 0]'; %x,y center of gravity, theta, lateral speed, yaw rate
cf = %front cornering stiffness coeff
cr = %rear cornering stiffness coeff

A = -(cf*cos(input)+cr)/(mass*speed);
B = (-lf*cf*cosInput+lr*cr)/(mass*speed)-speed;
C = (-lf*cf*cosInput+lr*cr)/(inertia*speed);
D = -(lf*lf*cf*cosInput+lr*lr*cr)/(inertia*speed);
E = cf*cosInput/mass;
F = lf*cf*cosInput/inertia;

vyDot = a*vy + c*r + e*input;
rDot = b*vy + d*r + f*input;
xDot = speed*cosTheta - vy*sinTheta;
yDot = speed*sinTheta + vy*cosTheta;
thetaDot = r;


initial_state = [1, 1, 0, 0, 0, 0, 0]'; %x,y center of gravity, theta, lateral speed, yaw rate
global_state = [1, 1, 0, 0, 0, 0, 0]'; %x y theta dx dy ax ay
dt = 0.01;
integral = zeros(2, 1);
