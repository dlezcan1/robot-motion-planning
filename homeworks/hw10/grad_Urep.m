%% grad_Urep.m
%
% gradient of repulsive potential field
%
% - written by: Dimitri Lezcano

function g_Urep = grad_Urep(q, CB, d_max)
    %% Arguments Block
    arguments
        q (2,1);
        CB cell;
        d_max double;
    end
    
    %% Iterate through all of the obstacles and sum grad_Urep
    g_Urep = zeros(size(q));
    for i = 1:numel(CB)
        % compute the closest point and the distance to the obstacle
       [c_pt_i, dist_i] = closest_point_obs(q, CB{i}); 
       
       % grad_direction
       g_dist = (q - c_pt_i) / dist_i;
       
       % compute the gradient and add to current
       if dist_i < d_max
           g_Urep = g_Urep + (1/d_max - 1/dist_i) * dist_i^(-2) * g_dist;
       end
       
    end
    
end

%% Helper functions
% compute the closest point on obstacle boundary
function [closest_pt, distance] = closest_point_obs(q, obs)
    arguments
        q (2,1);
        obs (2,:);
    end
    
    % check for overlapping structure
    if any(obs(:,1) ~= obs(:,end))
        obs = [obs, obs(:,1)];
    end
    
    N_edges = size(obs, 2) - 1;
    
    % iterate through the edges to get the closest_point
    distance = inf;
    for i = 1:N_edges
        % find closest point on the lineseg_i-i+1
        closest_pt_edge = closest_point_line(q, obs(:,i:i+1));
        
        % comptue the distance to the closest point
        distance_i = norm(closest_pt_edge - q);
        
        % check if the distance is smaller than the current
        if distance_i < distance
            distance = distance_i;
            closest_pt = closest_pt_edge;
        end
    end
    
end

function [closest_pt] = closest_point_line(q, lineseg)
    arguments
        q (2,1);
        lineseg (2,2);
    end
    
    l = lineseg(:,2) - lineseg(:,1);
    
    % check if linesegment is only a single point
    if norm(l) == 0
        closest_pt = lineseg(:,1);
        on_line = all(closest_pt == q);
        t = 0;
    else
        % compute the projection of q onto the line
        proj_q_l =  l * l'/norm(l)^2 * (q - lineseg(:,1));
        
        % add the projection to find the closest point
        closest_pt = lineseg(:,1) + proj_q_l;
        
        % what t parameter for the line does it fall on
        t = dot(l, closest_pt - lineseg(:,1))/dot(l,l);
        
        % check if it falls on the line segment
        on_line = (0 <= t) && (t <= 1);
        
        if t < 0
            closest_pt = lineseg(:,1);
        elseif t > 1
            closest_pt = lineseg(:,2);
        end
            
    end
    
end