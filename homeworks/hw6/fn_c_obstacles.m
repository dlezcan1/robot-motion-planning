%% fn_c_obstacles.m
%
% performs a minkowski difference for get the C-Space for obstacles
%
% - written by: Dimitri Lezcano

function [C_obs] = fn_c_obstacles(O_V, A0_V, q)
    %% Arguments Block
    arguments
        O_V cell;
        A0_V (2,:);
        q (3,1);
    end
    
    R = rotate2d(q(3));
    Aq_V = R * A0_V;
    
    C_obs = cell(size(O_V));
    for i = 1:numel(O_V)
        C_obs{i} = mink_sum(O_V{i}, -Aq_V);
    end

end

%% Helper Functions
function R = rotate2d(t)
    R = [cos(t) -sin(t); sin(t) cos(t)];
    
end