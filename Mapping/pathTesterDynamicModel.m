close all
State0 = zeros(13,1);


%bs values all SI
dt = 10*10^(-3);

m = 1;
lf = 0.15;
lr = 0.15;
K = 200; %N/M
C = 20;
Izz = 0.0086; % Assumed Cuboid Longitudinal = 0.3 Lateral = 0.115, m = 1kg
Iyy = 0.0086; %Assuming lateral aprroximately = height
Ixx = 0.002204; %With assumptions of Cuboid and lat = height = 0.115

options = [dt; m; lf; lr; K;C;Ixx;Iyy; Izz];

car = AutoRallyCar(State0,options);


wforce = 5; %Newton

%Currently Input in terms of tire forces

% f_{abc} a = fwd/back, b = left/right c = x,y dir

%u = [ steeringAngleRadians fflx ffrx ffly ffry fblx fbrx fbly fbry]


u = [deg2rad(5); wforce; wforce; 0; 0; wforce; wforce; 0; 0]; %example input with no side force

for t = 0:dt:1
    car = nextState(car,u);
end

comet(car.path(:,1),car.path(:,2))
grid on
xlabel('x in m')
ylabel('y in m')

figure
comet(car.path(:,4),car.path(:,5))
grid on
xlabel('Vx in m/s')
ylabel('Vy in m/s')

% format long
% plot(car.path(:,13), car.path(:,1));
% grid on
% hold on
% plot(car.path(:,13), car.path(:,4));
% 
% 
% legend('pos','vel')
% 
car.currentState
