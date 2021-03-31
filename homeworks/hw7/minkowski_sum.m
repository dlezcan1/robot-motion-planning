%% minkowski_sum.m
%
% this is a function to perform a minkowski sum for 2D shapes
%
% - written by: Dimitri Lezcano

function C = minkowski_sum(A, B)
    %% Arguments block
    arguments 
        A (2,:);
        B (2,:);
    end
    
    %% Compute all the sum of the vertices
    N_A = size(A, 2);
    N_B = size(B, 2);
    
    C_max = zeros(2, N_A * N_B);
    
    for i = 1:N_A
        for j = 1:N_B
            C_max(:,sub2ind([N_A, N_B], i, j)) = A(:,i) + B(:,j);            
        end
    end
    
    % convex hull
    k = convhull(C_max');
    C = C_max(:, reshape(k, 1, []));
    
end