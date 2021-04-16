function Q = mink_sum(A, B)
    %% Arguments Block
    arguments
        A (2,:)
        B (2,:)
    end
    
    %% sort A and B
    [yA_min, idx_A] = min(A(2,:));
    [yB_min, idx_B] = min(B(2,:));
    
    % check for ties in y-coord
    mask_A = (A(2,:) == yA_min);
    mask_B = (B(2,:) == yB_min);
    
    if sum(mask_A) > 1
        [~, idx_A] = min(A(1, mask_A));
    end
    
    if sum(mask_B) > 1
        [~, idx_B] = min(B(1,mask_B));
    end
   
    A = A(:, [idx_A:end, 1:idx_A-1]);
    B = B(:, [idx_B:end, 1:idx_B-1]);
   
    
    %% Algorithm
    i = 1; j = 1;
    N_A = size(A, 2); N_B = size(B, 2);
    
    A_tot = [A, A(:,1:2)]; B_tot = [B, B(:,1:2)];
    Q = [];
    
    while size(Q, 2) < N_A + N_B
        % grab each vector
        a_i = A_tot(:,i); b_j = B_tot(:,j);
        a_ip1 = A_tot(:,i+1); b_jp1 = B_tot(:,j+1);
        
        % add to Q sum
        Q = [Q, a_i + b_j];
        
        % compute angles
        ang_a = angle_2pts(a_i, a_ip1);
        ang_b = angle_2pts(b_j, b_jp1);
        
        % update rule
        if ang_a < ang_b
            i = i + 1;
        
        elseif ang_a > ang_b
            j = j + 1;
            
        else
            i = i + 1;
            j = j + 1;
        end
        
        if (i > N_A + 1) || (j > N_B + 1)
            break;
        end
        
    end    
end

%% Helper Functions
function theta = angle_2pts(v1, v2)
    dv = v2 - v1;
    theta = atan2(dv(2), dv(1));
    
    if theta < 0
        theta = theta + 2*pi;
    end

end