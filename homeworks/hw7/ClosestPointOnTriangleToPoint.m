%% ClosestPointOnTriangleToPoint.m
%
% this is a function to get the cloest Point on a triangle to a point
%
% - written by: Dimitri Lezcano

function [closest_pt] = ClosestPointOnTriangleToPoint(triangle, pt)
    %% Argument Block
    arguments
       triangle (2,3) {mustBeNumeric};
       pt (2,1) {mustBeNumeric} = [0;0];
    end
    
end