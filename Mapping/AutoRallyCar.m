classdef AutoRallyCar
    properties
        % x y z vx vy vz rollAng pitchAng yawAng roll pitch yaw time        
        currentState 
        
        % tstep m lf lr K C Af CD Ix Iy Iz
        options
        
        path
        
        stepCount
    end
    
    
    methods
        %Constructor Contains initial state 6D0F + 1 time derivative + t0
        function obj = AutoRallyCar(State0,options)
            
            obj.currentState = State0;
            obj.options = options;
            obj.path = [State0'];
            
            obj.stepCount = 0;
        end
        
        function [ax,ay,az,rolldd,pitchdd,yawdd] = nextAcceleration(obj,u)
            
            % Constants given to constructor
            dt = obj.options(1);
            m = obj.options(2);
            lf = obj.options(3);
            lr = obj.options(4);
            K = obj.options(5);
            C = obj.options(6);
            Af = obj.options(7);
            CD = obj.options(8);
            Ix = obj.options(9);
            Iy = obj.options(10);
            Iz = obj.options(11);
            
            rho = 1.125;
            g = 9.8;

            x = obj.currentState(1);
            y = obj.currentState(2);
            z = obj.currentState(3);
            vx = obj.currentState(4);
            vy = obj.currentState(5);
            vz = obj.currentState(6);
            rollAng = obj.currentState(7);
            pitchAng = obj.currentState(8);
            yawAng = obj.currentState(9);
            roll = obj.currentState(10);
            pitch = obj.currentState(11);
            yaw = obj.currentState(12);
                        
            
            %CONTROL INPUT (Vk) conversion assuming input is tire forces for u(2:end)
            
            del = u(1); %steering angle RADIANS
            
            fflx = u(2); %TIRE FORCES In newtons FOR NOW 
            ffrx = u(3);
            ffly = u(4); % f_{abc} a = fwd/back, b = left/right c = x,y dir
            ffry = u(5);
            fblx = u(6);
            fbrx = u(7);
            fbly = u(8);
            fbry = u(9);
            
            
            %No drag included
            
            %State AFTER time step using 1st Taylor Approx for Velocity 
            %and 2nd for Position
            
            
            %Cartesian acceleration terms
            ax = ( ( (fflx + ffrx)*cos(del) - (ffly + ffry)*sin(del) + ...
                fblx + fbrx)/m ) + vy*yaw - 0.5*CD*rho*Af*(vx)^2;
             
                    
            ay = (((fflx + ffrx)*sin(del) + (ffly + ffry)*cos(del) + ...
                fbly + fbry)/m) - vx*yaw;
            
            
            %Sprung mass vertical acceleration
            az = (-2*(2*K)*pitch - 2*(2*C)*vz + 2*(lf*K - lr*K)*rollAng ...
                + 2*(lf*C - lr*C)*pitch)/(m);
            
            
            
            %SondersConst
            
            wf = 0;
            wr = 0;
            
            hs = 0.110; %Sprung height
            hc = 0.115; %Height of rolling pivot
            
            Zs = 0.115;
            
            
            %Angular Accelerations
            
            r1  = ( (-1*(wf*wf*K + wr*wr*K)) /2); %Spring Stiffness
            
            r2 = ((-1*(wf*wf*C +wr*wr*C))/2); %Dashpot
            
            r3 = m*g*sin(rollAng) + m*ay*cos(rollAng); %Due to acceleration from input
            
            
            rolldd = ((r1*rollAng + r2*roll + r3*(hs - hc))/(Ix)); %roll acceleration at step due to input
            
            
            p1 = 2*(lf*K - lr*K);
            
            p2 = 2*(lf*C - lr*C);
            
            p3 = -2*(lf*lf*K + lr*lr*K);
          
            p4 = -2*(lf*lf*C + lr*lr*C);
            
            p5 = m*g*sin(pitchAng) + m*ax*cos(pitchAng);
            
            pitchdd = (p1*Zs + p2*vz + ...
                p3*pitchAng + p4*pitch ...
                 + p5*hs )/(Iy); %pitch acceleration at step due to input
            
            
            yawdd = ( (((ffly + ffry)*cos(del) + (fflx + ffrx)*sin(del))*lf...
                 - (fbly + fbry)*lr)/Iz ); %yaw acceleration at step due to input
            
            
            
            
        end

        function obj = nextState(obj, u) %input vector
            
            % Constants given to constructor necessary here
            dt = obj.options(1);

            
            g = 9.8;
            
            
            px = obj.currentState(1);
            py = obj.currentState(2);
            pz = obj.currentState(3);
            vx = obj.currentState(4);
            vy = obj.currentState(5);
            vz = obj.currentState(6);
            rollAng = obj.currentState(7);
            pitchAng = obj.currentState(8);
            yawAng = obj.currentState(9);
            roll = obj.currentState(10);
            pitch = obj.currentState(11);
            yaw = obj.currentState(12);
             
             
            [ax,ay,az,rolldd,pitchdd,yawdd] = nextAcceleration(obj,u); 
             
            if pz > 0 
                az = az - g; %adding effect of gravity and making the assumption sprung height actual height
            end
            
             
             %1st Order taylor series expansion of translation terms
            
            
            px = px + vx*dt + ax*(dt*dt); %here the velocity is updated following the position
            
            vx = vx + ax * dt; % so prior state velocity and posterior state acceleration are used

            
            py = py + vy*dt + ay*(dt * dt);
           
            vy = vy + ay * dt;
            
                        
            pz = pz + vz*dt + az*(dt * dt);
            
            vz = vz + az * dt;

            
            %offset from center of mass to ground:
            
            hCMoffset = 0.02;
            
            if pz < -hCMoffset
                pz = -hCMoffset; %Maintaining a solid level ground
            end
              
            
            %Integration of angles
            
            roll = roll + rolldd*dt;
            
            rollAng = rollAng + roll*dt + rolldd*(dt * dt);
            
            pitch = pitch + pitchdd*dt;
            
            pitchAng = pitchAng + pitch*dt + pitchdd*(dt*dt);
            
            yaw = yaw + yawdd*dt;
            
            yawAng = yawAng + yaw*dt + yawdd*(dt * dt);
            
                        
            
            %for 2D case
%             rollAng = 0;
%             pitchAng = 0;
%             roll = 0;
%             pitch = 0;
%             pz = 0;
%             vz = 0;
            
            
            % Now updating state info
            
            obj.currentState(1) = px;
            obj.currentState(2) = py;
            obj.currentState(3) = pz;
            obj.currentState(4) = vx;
            obj.currentState(5) = vy;
            obj.currentState(6) = vz;
            obj.currentState(7) = rollAng;
            obj.currentState(8) = pitchAng;
            obj.currentState(9) = yawAng;
            obj.currentState(10) = roll;
            obj.currentState(11) = pitch;
            obj.currentState(12) = yaw;
            obj.currentState(13) = obj.currentState(13) + dt;
            
            obj.stepCount = obj.stepCount + 1;
            
           %Adding current State to path
            obj.path = [obj.path; obj.currentState']; %Each step is ROW
            
        end
        
        
    end
    
end
