%% GJKalg_2D.m
%
% This is a function to implement the 2D version of the 
% GJK algorithm
%
% - written by: Dimitri Lezcano

function [distance] = GJKalg_2D(A, B)
    %% Argument Block
    arguments
        A (2,:) {mustBeNumeric};
        B (2,:) {mustBeNumeric};
    end
    
    %% Set-up
    N = 100; % maximum number of iterations
    
    %% Compute Minkowski Difference
    C = minkowski_sum(A, -B);
    
    %% GJK Algorithm
    % initialization
    V = C(:,1:3);
    
    % iterations
    for k = 1:N
        [p_k, vor_k, edge_k] = ClosestPointOnTriangleToOrigin(V);
        
        % return 0 if we have the highest return
        if all(p_k == 0) || iselement(V, [0;0])
            distance = 0;
            return
        end
        
        % find smallest Q_k containing p_k
        if vor_k.a
            Q_k = V(:,1:2);
        elseif vor_k.b
            Q_k = V(:,2:3);
        elseif vor_k.c
            Q_k = V(:,[3,1]);
        elseif edge_k.ab
            Q_k = V(:,1:2);
        elseif edge_k.bc
            Q_k = V(:,2:3);
        elseif edge_k.ca
            Q_k = V(:,[3,1]);
        end
        
        % find the supporting point in C
        q_k = support_point(C, p_k);
        
        % return if q_k in Q_k
        if iselement(Q_k, q_k)
            distance = norm(p_k);
            return
        end
        
        % repeat
        V = [Q_k, q_k];
        
    end
       
        
end

%% Helper functions
function retval = iselement(A, p)
    arguments
        A (2,:);
        p (2,1);
    end
    
    retval = any(ismember(A', p', 'rows'));
end

function s_pt = support_point(A, p)
    arguments
        A (2,:);
        p (2,1);
    end
    
    % find which points lie along -p direciton
    support_direction = ((A - p)'*(p));
    
    % pick a supporting point
    k = find(support_direction <= 0); % no or some decrease
    [~, midx] = min(support_direction);
    s_pt = A(:,midx);
end