%% generate_Sn_line.m
%
% function to generate a line in the n-D torus
%
% - written by: Dimitri Lezcano

function q_line = generate_Sn_line(q1, q2, dq, options)
    %% Arguments block
    arguments
        q1 (:,1);
        q2 (:,1);
        dq double;
        options.wraparound logical = true;
    end
    
    d_nowrap = q2 - q1;
    
    % check if line can wrap around [0,2*pi)
    if options.wraparound
        % check if [-pi,pi) if closer in distance
        d_wrap = Sn_bounds(q2, true) - Sn_bounds(q1, true);
        
        d_both = [d_nowrap, d_wrap];
        
        % find closest distance direction
        [~, idxs] = min(abs(d_both), [], 2);       
        d_closest = d_both(sub2ind(size(d_both), (1:4)', idxs));
        
        % generate new line
        q_line = q1 + (0:dq:1).*d_closest;
    else
        % basic line
        q_line = q1 + (0:dq:1).*d_nowrap;
    end
    
%     % rebound the points (Not sure if needed)
%     q_line = Sn_bounds(q_line, false);
end
