%% grad_Uatt.m
%
% function to compute the gradient of the attraction potential
%
% - written by: Dimitri Lezcano

function g_Uatt = grad_Uatt(q, qG, d_min)
    %% Arguments block
    arguments
        q (:,1);
        qG (:,1);
        d_min = 0.01;
    end
    %% Compute distance and gradient
    dist_q = norm(q - qG);
    
    g_Uatt = q - qG;
    
end