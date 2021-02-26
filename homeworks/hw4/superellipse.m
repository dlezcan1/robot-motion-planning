%% superellipse.m
%
% this is a function for generating super-ellipse boundary points in 2-D
%
% - written by: Dimitri Lezcano


function boundary = superellipse(t, a1, a2, epsilon)
    %% Arguments block
    arguments
        t (1,:) {mustBeNumeric};
        a1 double {mustBeGreaterThan(a1, 0)};
        a2 double {mustBeGreaterThan(a2, 0)};
        epsilon double {mustBeGreaterThan(epsilon, 0)};
    end
    
    %% Parameterizations
    % first quadrant
    
    x1 = a1 * sign(cos(t)) .* abs(cos(t)).^epsilon;
    y1 = a2 * sign(sin(t)) .* abs(sin(t)).^epsilon;
    
    boundary = [x1; y1];
    
    
    
   
    
    
end