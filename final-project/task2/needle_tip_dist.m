%% needle_tip_dist.m
%
% function to compute distance between configurations for needle tip
% Cancels out orientation.
%
% - written by: Dimitri Lezcano

function dist = needle_tip_dist(q, Q)
    arguments
        q (3,1);
        Q (3,:);
    end
    
    dq = [1;1;0].*(q - Q); % cancel out orientation. We don't care
    
    dist = vecnorm(dq, 2, 1);
end