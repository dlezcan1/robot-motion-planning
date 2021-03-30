%% ClosestPointOnTriangleToPoint.m
%
% this is a function to get the cloest Point on a triangle to a point
%
% - written by: Dimitri Lezcano

function [closest_pt, vor, edge] = ClosestPointOnTriangleToPoint(triangle, pt)
    %% Argument Block
    arguments
       triangle (2,3) {mustBeNumeric};
       pt (2,1) {mustBeNumeric} = [0;0];
    end
    
   %% Calculation
   [closest_pt_origin, vor, edge] = ClosestPointOnTriangleToOrigin(triangle - pt);
   closest_pt = closest_pt_origin + pt;
   
end