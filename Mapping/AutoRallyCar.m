classdef AutoRallyCar
    properties
        % x y z vx vy vz rollAng pitchAng yawAng roll pitch yaw time        
        currentState 
        
        % tstep m lf lr K C Ix Iy Iz
        options
        
        path
        
        stepCount
    end
    
    
    methods
        %Constructor Contains initial state 6D0F + 1 time derivative + t0
        function obj = AutoRallyCar(State0,options)
            
            if (size(State0) ~= 13) || (size(options) ~= 9)
                error('Vector lengths incorrect')
            end
            
            obj.currentState = State0;
            obj.options = options;
            obj.path = [];
            
            obj.stepCount = 0;
        end
        
        function obj = nextStep(obj, u) %input vector
            
            %Adding current State to PAST path
            obj.path = [obj.path; obj.currentState']; %Each step is ROW
            
            % Constants given to constructor
            tstep = obj.options(1);
            m = obj.options(2);
            lf = obj.options(3);
            lr = obj.options(4);
            K = obj.options(5);
            C = obj.options(6);
            Ix = obj.options(7);
            Iy = obj.options(8);
            Iz = obj.options(9);
            

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
            
            ax = ( ( (fflx + ffrx)*cos(del) - (ffly + ffry)*sin(del) + ...
                fblx + fbrx)/m ) + ...
                obj.currentState(5)*obj.currentState(12);
            
            vx = obj.currentState(4) + ax * tstep;
            
            px = obj.currentState(1) + vx*tstep + ax*(tstep*tstep);
            
            
            ay = (((fflx + ffrx)*sin(del) + (ffly + ffry)*cos(del) + ...
                fbly + fbry)/m) - obj.currentState(4)*obj.currentState(12);
            
            vy = obj.currentState(5) + ay * tstep;
            
            py = obj.currentState(2) + vy*tstep + ay*(tstep * tstep);
            
            rdot = ( (((ffly + ffry)*cos(del) + (fflx + ffrx)*sin(del))*lf...
                 - (fbly + fbry)*lr)/Iz ); %yawdd
            
            yaw = obj.currentState(12) + rdot*tstep;
            
            yaw_ang = obj.currentState(9) + yaw*step + rdot*(tstep * tstep);
            
            
            %for 2D case
            roll_ang = 0;
            pitch_ang = 0;
            roll = 0;
            pitch = 0;
            pz = 0;
            vz = 0;
            
            
            % Now updating state info
            
            obj.currentState(1) = px;
            obj.currentState(2) = py;
            obj.currentState(3) = pz;
            obj.currentState(4) = vx;
            obj.currentState(5) = vy;
            obj.currentState(6) = vz;
            obj.currentState(7) = roll_ang;
            obj.currentState(8) = pitch_ang;
            obj.currentState(9) = yaw_ang;
            obj.currentState(10) = roll;
            obj.currentState(11) = pitch;
            obj.currentState(12) = yaw;
            obj.currentState(13) = obj.currentState(13) + tstep;
            
            
            
            obj.stepCount = obj.stepCount + 1;
            
        end
        
        
        
        
    end
    
end
