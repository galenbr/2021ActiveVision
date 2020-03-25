classdef cube
    
    properties
        size; u_s; u_r; mass; F; mg; piercing_angles;
    end  % properties
    
    methods
%% Constructor
        function obj = cube()
            obj.size = 2;
            obj.u_s = 0.25;
            obj.u_r = 0.1;
            obj.mass = 0.05;
            obj.F = 1;
            obj.mg = obj.mass*10;
            obj.piercing_angles = [15,30,45,60,90];
        end  % function
%% Generate Force Function
        function vectors = generate_force(obj)
         for i = 1:length(obj.piercing_angles)
             vec = [sind(obj.piercing_angles(i)), 0 , -cosd(obj.piercing_angles(i))];
             vectors(i,:) = vec;
         end
        end  % function
%% Simulation Function        
        function simulator(obj,vectors)
            force_vectors = obj.F*vectors; % Decompose force vectors along co-ordinate axes
            
            %% Compute forces
            for i = 1:length(force_vectors)
                static_friction = [-obj.u_s*obj.mg+obj.u_s*force_vectors(i,3),0,0]; %Assuming friction is always in -x since block is always pushed in +x
            if(force_vectors(i,1)>-static_friction(1))
                % Cube Slides
                sliding_friction = [-obj.u_r*obj.mg+obj.u_r*force_vectors(i,3),0,0]; % now under effect of sliding friction
                resultant_force = force_vectors(i) + sliding_friction;
                acceleration = resultant_force/obj.mass;
            else
                % Cube does not slide
                acceleration = [0,0,0];
            end
            %% Simulate motion of cube for t = 0 to t = 2
            initial_vel = 0;acceleration_x = acceleration(1);
            %center_cube = [1,1,2];
            if(acceleration_x == 0)
                end_time = 0.4;
            else
                end_time = 1;
            end
            for time = 0:0.2:end_time
                clf('reset');
                figure(1)
                set(gcf,'Position',[0, 0, 1000,1000])
                title("Trial:"+i)
                axis([0 20 -15 15 0 20])
                grid on
                hold on
                % vel_x = initial_vel + acceleration_x*time;
                s_x = initial_vel*time + acceleration_x*(time^2);
                center_cube = [s_x, 1, 2];
                q = quiver3(center_cube(1),center_cube(2),center_cube(3),vectors(i,1),vectors(i,2),vectors(i,3));
                q.LineWidth = 5;
                plotcube((obj.size*[1,1,1]),[s_x,0,0],0.8,[1,0,0]);
                pause(.5)
            end
            end
        end  % function
%% Max Piercing Angle Function
        function max = max_piercing_angle(obj)
            slide = false;
            piercing_angle = 0; max = 0;
            % Need Cube and force properties

            %while(~slide)
                % Decompose Forces ie Calculate Force Vectors
                % Calculate Static Friction
    
                %if(force_vectors(i,1)>-static_friction(1))
                    %slide = true;
                %else
                %    continue;
                %end
            %end
        
        end  % function
    end  % methods
end  % classdef