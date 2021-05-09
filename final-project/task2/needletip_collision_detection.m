%% needletip_collision_detection.m
%
% function for needletip collision detection
%
% - written by: Dimitri Lezcano

function colliding = needletip_collision_detection(q, obs)
    arguments
        q (3,1);
        obs cell;
    end
    
    colliding = false;
    for i = 1:numel(obs)
        obs_i = obs{i};
        
        % check if tip is in polygon
        in_i = inpolygon(q(1), q(2), obs_i(1,:), obs_i(2,:));
        
        % return if it is
        if in_i
            colliding = true;
            return;
        end
    end
end