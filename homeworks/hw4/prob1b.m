%% prob1b.m
%
% this is script to run through problem 1b
%
% - written by: Dimitri Lezcano

function prob1b
    %% Set-Up
    % super-ellipse params
    a1 = 2
    a2 = 1
    epsilons = [0.2, 0.5, 1, 2, 3, 4];
    
    % boundary length
    t = linspace(0, 2*pi, 100);
    
    %% Iterate over the epslions
    fig = figure(1);
    
    for e = epsilons
        % get the boundary
        boundary = superellipse(t, a1, a2, e);
        
        % plot the boundary
        plot(boundary(1,:), boundary(2,:),'-',...
            'LineWidth', 2, 'DisplayName', "\epsilon = " + e); hold on;
        axis equal;
        
    end
    hold off;
    legend();
    

end


end