function next_state = integraterKinematic(initial_state, dt, length, velocity, steering_angle)
velocity_x = velocity * cos(initial_state(3));
velocity_y = velocity * sin(initial_state(3));
velocity_theta = velocity / length * tan(steering_angle);

delta_x = velocity_x * dt;
delta_y = velocity_y * dt;
delta_theta = velocity_theta * dt;

next_state = [initial_state(1:3), 0, 0] + [delta_x, delta_y, delta_theta, velocity, 0];
end

