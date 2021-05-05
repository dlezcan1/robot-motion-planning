%% rot2d.m
%
% function for 2d rotation matrix
%
% - written by: Dimitri Lezcano

function R = rot2d(t)
    arguments 
        t double;
    end
    
    R = [cos(t) -sin(t); sin(t) cos(t)];
end