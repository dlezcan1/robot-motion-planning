%% prob2a.m
%
% this is a function to test homeomorphism
% 
% - written by: Dimitri Lezcano

%% Main function
function pts = prob2a(t)
    arguments
        t (1,:) = linspace(0, 2*pi, 100);
    end
    % break up the t's
    t1 = t(0 <= t & t < pi/2);
    t2 = t(pi/2 <= t & t < pi);
    t3 = t(pi <= t & t < 3*pi/2);
    t4 = t(3*pi/2 <= t & t < 2*pi);
    
    % map each
    pts1 = [1/sqrt(2) * ones(size(t1));
            sin(t1 - pi/4)];
    pts2 = [cos(t2 - pi/4);
            1/sqrt(2)* ones(size(t2))];
    pts3 = [-1/sqrt(2)* ones(size(t3));
            sin(t3 - pi/4)];
    pts4 = [cos(t4 - pi/4);
            -1/sqrt(2)* ones(size(t4))];
        
    
    
    pts = [ pts1, pts2, pts3, pts4];
    
end
            