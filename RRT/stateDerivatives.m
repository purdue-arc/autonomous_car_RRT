function [derivatives] = stateDerivatives(state, length, width, mass, vx, cf, cr, lf, lr, steeringAngle)

    theta = state(3);
    vy = state(4);
    %Values for non-linear model of vydot and r(yaw rate)
    A = -(cf*cos(steeringAngle)+cr)/(mass*vx);
    B = (-lf*cf*cos(steeringAngle)+lr*cr)/(mass*vx)-vx;
    C = (-lf*cf*cos(steeringAngle)+lr*cr)/(inertia*vx);
    D = -(lf*lf*cf*cos(steeringAngle)+lr*lr*cr)/(inertia*vx);
    E = cf*cos(steeringAngle)/mass;
    F = lf*cf*cos(steeringAngle)/inertia;
    
    vyDot = A*vy + C*r + E*input;
    rDot = B*vy + D*r + F*input;
    xDot = speed*cos(theta) - vy*sin(theta);
    yDot = speed*sin(theta) + vy*cos(theta);
    thetaDot = r;
    
    derivatives = [xDot, yDot, thetaDot, vyDot, rDot];
    
end
