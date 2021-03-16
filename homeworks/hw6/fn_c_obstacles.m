%% fn_c_obstacles.m
%
% performs a minkowski difference for get the C-Space for obstacles
%
% - written by: Dimitri Lezcano

function [C_obs] = fn_c_obstacles(O_V, A0_V, q)
    %% Arguments block
    arguments
       O_V (2,:);
       A0_V (2,:);
       q (3,1);
    end
    
    %% Set-up
    % sorting
    O_V = sortrows(O_V', [2,1])';
    A0_V = -sortrows(-A0_V', [2,1])';
    % number grabbing
    N_O = size(O_V, 2);
    N_A0 = size(A0_V, 2);
    O_V = [O_V, O_V(:,1:2)]; % Wrap-arounds
    A0_V = [A0_V, A0_V(:,1:2)]; % Wrap-arounds
    
    %% Minkowski Difference Algorithm
    Q = []; i = 1; j = 1;
    while size(Q, 2) < N_O + N_A0
        % Grab the vector pairs
        a_i = O_V(:,i); b_j = -A0_V(:,j); % a_i, b_j
        a_ip1 = O_V(:, i+1); b_jp1 = -A0_V(:,j+1); % a_i+1, b_j+1
        
        % add the next point
        Q = [Q, a_i + b_j];
        
        % compute the angles
        ang_a = angle_2pts(a_i, a_ip1);
        ang_b = angle_2pts(b_j, b_jp1);
        
        % find which point to iterate
        if ang_a < ang_b
            i = i+1;
            
        elseif ang_a > ang_b
            j = j+1;
            
        else
            i = i+1; j = j+1;
        end
        
        % Make sure indices are in bounds
        if (i > N_O + 1) || (j > N_A0 + 1)
            break
        end
        
    end
    C_obs = Q;
%     % remove collinear point
%     Q_ext = [Q, Q(:,1)];
%     C_obs = [Q(:,1)];
%     for i = 2:size(Q, 2)
%         c_im1 = Q_ext(:,i-1);
%         c_i = Q_ext(:,i);
%         c_ip1 = Q_ext(:,i+1);
%         
%         v_i_im1 = c_i - c_im1;
%         v_ip1_i = c_ip1 - c_i;
%         
%         ang_collin = angle_2vecs(v_i_im1, v_ip1_i);
%         
%         collinear_test = abs(ang_collin - pi)<1e-4 | abs(ang_collin + pi) < 1e-4;
%         
%         if ~collinear_test
%             C_obs = [C_obs, c_i];
%         end
%     end

    
    
end


%% Helper Functions
function theta = angle_2pts(v1, v2)
    dv = v2 - v1;
    theta = atan2(dv(2), dv(1));
    
    if theta < 0
        theta = theta + 2*pi;
    end

end

function R = rotate2d(t)
    R = [cos(t) -sin(t); sin(t) cos(t)];
    
end