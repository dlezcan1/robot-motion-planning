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
    qI = zeros(6,1);
    qG = [pi/2; pi/4; 19*pi/10; pi/6];
    
    % obstacles
    obs = {};
    




end

%% Helper functions
% robot functions


% helper plot functions    
function plot_robot(q, link_len)
    arguments
        q (:,1);
        link_len double;
    end
    
    robot_joints = get_robot_joints(q, link_len);
    
    plot(robot_joints(1,:), robot_joints(2,:), 'k*');
end

function plot_obstacles(obs)
    arguments
        obs cell;
    end
    
    for i = 1:numel(obs)
        obs_i = obs{i};
        
        patch(obs_i(1,:), obs_i(2,:), 'red');
    end
end
        