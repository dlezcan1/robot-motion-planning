%% brute_minkowski_diff.m
%
% brute force minkowski difference

function [C_obs] = brute_minkowski_diff(O_V, A0_V)
    arguments
        O_V (2,:)
        A0_V (2,:)
    end

    O_V = [O_V, O_V(:,1)];
    A0_V = [A0_V, A0_V(:,1)];
    
    N_O = size(O_V, 2);
    N_A0 = size(A0_V, 2);
    
    Min_Diff = zeros(2, N_O * N_A0);
    for i = 1:N_O
        for j = 1:N_A0
            Min_Diff(:,sub2ind([N_O, N_A0], i, j)) = O_V(:,i) - A0_V(:,j);
            
        end
    end

    k = convhull(Min_Diff');
    C_obs = Min_Diff(:, reshape(k, 1, []));
    
end