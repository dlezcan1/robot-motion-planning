%% main_c_obstacles.m
%
% main script to test the function "fn_c_obstacles.m"
%
% - written by: Dimitri Lezcano

%% Set-Up
%  problem 5 as an example
O_V = [0, 2, 1;
       0, 0, sqrt(3)];
A0_V = O_V;

%% Algorithms
custom_C_obs = fn_c_obstacles(O_V, A0_V, [0;0;0]);
brute_C_obs = brute_minkowski_diff(O_V, A0_V);

%% Plotting
custom_idxs = [1:size(custom_C_obs, 2), 1];
brute_idxs = [1:size(brute_C_obs, 2), 1];

fig = figure(1);
plot(custom_C_obs(1, custom_idxs),custom_C_obs(2, custom_idxs), '*-', ...
    'DisplayName', 'custom', 'LineWidth', 2); hold on;
plot(brute_C_obs(1, brute_idxs),brute_C_obs(2, brute_idxs), '*--', ...
    'DisplayName', 'brute-force', 'LineWidth', 2); hold off;
legend('Location', 'bestoutside'); axis square;
xlabel('x'); ylabel('y');
title("Problem 6");

%% Saving
saveas(fig, 'prob6b.png'); 
disp("Saved figure: 'prob6b.png'");

%% Helper Functions
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