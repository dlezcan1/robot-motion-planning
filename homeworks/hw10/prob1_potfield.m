m%% prob1_potfield.m
%
% this runs the potential field method on problem 1
%
% - written by: Dimitri Lezcano

function prob1_potfield
    %% Set-Up
    % obstacle boundaries
    CB{1} = [ 0 50 50  0; 25  25  50 50];
    CB{2} = [80 80 70 70; 50 100 100 50];


    % initial and final configurations
    qI = [0.5; 0.5]; qG = [95; 95];
    
    % constants for the method
    step_size = .5;
    c_att = .005; c_rep = 15;
    N_max = 3000;
    d_max = 25;
    end_epsilon = 1e-4; 
    
    % gradient function 
    grad_U = @(q) c_att*grad_Uatt(q, qG) + c_rep*grad_Urep(q,CB,d_max); 
    
    %% Perform the potential field descent
    q = [qI];
    i = 1;
    grads = [grad_U(q)];
    
    while (norm(grads(:,i)) > end_epsilon) && (i <= N_max)
        % grab the q_i
        q_i = q(:,i);
        grad_i = grads(:,i);
        
        % compute the update from the previous gradient
        q_ip1 = q_i - step_size * grad_i;
        
        % compute the new gradient
        grad_ip1 = grad_U(q_ip1);
        
        % append new step and iterate
        q = [q, q_ip1];
        grads = [grads, grad_ip1];
        i = i + 1;
    end
    
    %% Plot the results
	close all;
    fig = figure(1);
    hold off;
    for i = 1:numel(CB)
        cb = CB{i};
        patch(cb(1,:), cb(2,:), 'red'); hold on;
    end
    
    plot(q(1,:), q(2,:), 'k.', 'DisplayName', 'path'); hold on;
    plot(qI(1), qI(2), 'b*', 'DisplayName', 'start'); hold on;
    plot(qG(1), qG(2), 'g*', 'DisplayName', 'goal'); hold on;
    hold off;
    legend('Location', 'bestoutside'); axis equal; grid on; 
    xlabel('x'); ylabel('y');
    title("Homework 10: Problem 1");
    
    
    %% Save the results
    saveas(fig, 'prob1.png');

end