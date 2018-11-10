function next_state = integrater(initial_state, dt, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle)
%Fourth order Runge-Kutta to get next state
k1 = stateDerivatives(initial_state, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);
k2 = stateDerivatives(initial_state + k1./2, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);
k3 = stateDerivatives(initial_state + k2./2, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);
k4 = stateDerivatives(initial_state + k3, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);

next_state = initial_state + dt/6*(k1 + 2*k2 + 2*k3 + k4);

end
