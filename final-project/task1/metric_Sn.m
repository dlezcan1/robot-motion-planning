%% metric_Sn
%
% this is a custom metric function on Sn
%
% - written by: Dimtiri Lezcano

function dist = metric_Sn(q1, q2, options)
    arguments
        q1 (:,1);
        q2 (:,1);
        options.wraparound logical = true;
    end
    
    d_fwd = Sn_bounds(q2, false) - Sn_bounds(q1, false);    % forward  distance
    
    if options.wraparound
        d_bwd = Sn_bounds(q2, true) - Sn_bounds(q1, true);  % backward distance
    
        % find closest distances
        d_closest = min(abs(d_fwd), abs(d_bwd));
        
    else % only do forward calculation
        d_closest = d_fwd;
    end

    % L1 norm
    dist = sum(d_closest);
    
end