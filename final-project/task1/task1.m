%% task1.m
%
% main script for RMP n-link planar serial manipulator
%
% - written by: Dimitri Lezcano

function task1
    %% Set-up
    % robot parameters
    S.n = 4;
    S.link_len = 1;
    
    
    % initial and goal configurations
    qI = zeros(S.n,1);
    qG = [pi/2; -pi/3; -pi/3; pi/2.5];
    
    % obstacles
    obs = {[1/2, 1, 1, 1/2;
            1/2, 1/2, 1, 1]};
    obs{2} = -1.5*obs{1};
    obs{3} = [1, 1.5, 2, 2, 1
              2, 1.8, 2, 3, 3];
          
    S.obs = obs;
    
    %% Check collision detection
    q_test = [pi/10; pi/4; pi/4; pi/4];
    q_test = [pi/4; 0; 0; 0];
    
    disp("Collision Detection:")
    colliding = collision_detection(q_test, S);
    
    if colliding
        disp("Colliding");
    else
        disp("Collsiion-Free");
    end          
          
    %% Plotting
    fig = figure(1);
    plot_robot(qI, S.link_len, 'linespec', 'b-o'); hold on;
    plot_robot(q_test, S.link_len); hold on;
    plot_robot(qG, S.link_len, 'linespec', 'g-o'); hold on;
    plot_obstacles(obs, 'color', 'black');
    grid on;
    xlabel('x'); ylabel('y');
    hold off;

end

%% Helper functions
% helper plot functions    
function plot_robot(q, link_len, options)
    arguments
        q (:,1);
        link_len double;
        options.linespec = 'k-o';
    end
    ax_lim = link_len * numel(q);
    robot_joints = get_robot_joints(q, link_len);
    
    plot(robot_joints(1,:), robot_joints(2,:), options.linespec);
    xlim([-ax_lim, ax_lim]); ylim([-ax_lim, ax_lim]);
end

function plot_obstacles(obs, options)
    arguments
        obs cell;
        options.color = 'red';
    end
    
    for i = 1:numel(obs)
        obs_i = obs{i};
        
        patch(obs_i(1,:), obs_i(2,:), options.color);
    end
end
        