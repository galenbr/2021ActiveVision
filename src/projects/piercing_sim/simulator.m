%% Cube properties
cube_size = 2; % Cube edge length
u_s = 0.25; % static friction co-efficient
u_r = 0.1; % sliding friction co-efficient
mass = .05; % mass of the cube in kg
mg = mass*10; % weight of the cube in N
F = 1; % magnitude of force applied in N
vectors = generate_force(); % Generate different force vectors
force_vectors = F*vectors; % Decompose force vectors along co-ordinate axes

%% Compute forces
for i = 1:length(force_vectors)
    static_friction = [-u_s*mg+u_s*force_vectors(i,3),0,0]; %Assuming friction is always in -x since block is always pushed in +x
if(force_vectors(i,1)>-static_friction(1))
    % Cube Slides
    sliding_friction = [-u_r*mg+u_r*force_vectors(i,3),0,0]; % now under effect of sliding friction
    resultant_force = force_vectors(i) + sliding_friction;
    acceleration = resultant_force/mass;
else
    % Cube does not slide
    acceleration = [0,0,0];
end

%% Simulate motion of cube for t = 0 to t = 2
initial_vel = 0;acceleration_x = acceleration(1);
center_cube = [1,1,2];
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
    vel_x = initial_vel + acceleration_x*time;
    s_x = initial_vel*time + acceleration_x*(time^2);
    center_cube = [s_x, 1, 2];
    q = quiver3(center_cube(1),center_cube(2),center_cube(3),vectors(i,1),vectors(i,2),vectors(i,3));
    q.LineWidth = 5;
    plotcube((cube_size*[1,1,1]),[s_x,0,0],0.8,[1,0,0]);
    pause(.5)
end
end