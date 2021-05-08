%% isintersect_linepolygon.m
%
% this is a function for collision checking in R2 for a convex polygon
%
% Args:
%   - P, polygon (2,N) points
%   - q_line, (2,2) [q0, q1] line segment of interest (points are along the columns)
%
% Return:
%   - boolean true if intersects the polygon, otherwise false
%
% - written by: Dimitri Lezcano

function [does_intersect, tE, tL] = isintersect_linepolygon(P, q_line)
    %% arugments block
    arguments
        P (2,:);
        q_line (2,2);
    end
    
    %% unpack the arguments
    % unpack line points
    q0 = q_line(:,1);
    q1 = q_line(:,2);
    
    % determine -90 deg rotation
    R = [0 1; -1 0];
    P = P(:, [1:end, 1]);
    
    %% Check for single point
    if all(q0 == q1)
        does_intersect = inpolygon(q0(1), q0(2), P(1,:), P(2,:));
        return;
    
    %% Check for a line segment
    else
        % initialization
        tE = 0; tL = 1;
        ds = q1 - q0;
        
        % iterate through the edges
        for i = 1:size(P, 2)-1
            % grab the edge and outward normal
            e_i = P(:,i+1) - P(:,i);
            n_i = R * e_i;
            
            % Computational test
            N = -dot(q0 - P(:,i), n_i);
            D = dot(ds, n_i);
            
            % first check (D = 0 | approximation)
            if abs(D) < 1e-10 && N < 0
                does_intersect = false;
                return;
            end
            
            % update and second check
            t = N/D;
            
            if D < 0
                tE = max(tE, t);
                if tE > tL
                    does_intersect = false;
                    return;
                end
                
            else % D > 0
                tL = min(tL, t);
                if tL < tE
                    does_intersect = false;
                    return;
                end
            end
            
        end
           
        % last check
        if tE <= tL
            does_intersect = true;
            return;
        else
            does_intersect = false;
            return;
        end
    end
    
end