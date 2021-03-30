%%  ClosestPointOnTriangleToOrigin.m
%
% this is a function to get the cloest Point on a triangle to the origin
%
% - written by: Dimitri Lezcano

function [closest_pt, vor, edge] = ClosestPointOnTriangleToOrigin(triangle)
    %% Argument Block
    arguments
       triangle (2,3) {mustBeNumeric};
    end
    
    %% Get edge vectors (from the first)    
    line_ab = triangle(:,2) - triangle(:,1); line_ba = -line_ab;
    line_ac = triangle(:,3) - triangle(:,1); line_ca = -line_ac;
    line_bc = triangle(:,3) - triangle(:,2); line_cb = -line_bc;
    
    %% Determine if origin is in Voronoi Regions
    vor.a = (dot(line_ab, -triangle(:,1)) <= 0) & (dot(line_ac, -triangle(:,1)) <= 0);
    vor.b = (dot(line_bc, -triangle(:,2)) <= 0) & (dot(line_ba, -triangle(:,2)) <= 0);
    vor.c = (dot(line_ca, -triangle(:,3)) <= 0) & (dot(line_cb, triangle(:,3)) <= 0);
            
    %% Determine if origin is in edge regions
    % E_AB
    edge.ab = (dot_wrap(cross_wrap(cross_wrap(line_bc, line_ba), line_ba), -triangle(:,2)) >= 0) ...
        & (dot_wrap(-triangle(:,1), line_ab) >= 0) & (dot_wrap(-triangle(:,2), line_ba) >= 0);
    
    % E_bc
    edge.bc = (dot_wrap(cross_wrap(cross_wrap(line_ca, line_cb), line_cb), -triangle(:,3)) >= 0) ...
        & (dot_wrap(-triangle(:,2), line_bc) >= 0) & (dot_wrap(-triangle(:,3), line_cb) >= 0); % a -> b -> c -> a
    
    % E_ca
    edge.ca = (dot_wrap(cross_wrap(cross_wrap(line_ab, line_ac), line_ac), -triangle(:,1)) >= 0) ...
        & (dot_wrap(-triangle(:,3), line_ca) >= 0) & (dot_wrap(-triangle(:,1), line_ac) >= 0); % a -> b -> c -> a
                 
    %% Compute distance
    % vors
    if vor.a
        closest_pt = triangle(:,1);
    elseif vor.b
        closest_pt = triangle(:,2);
    elseif vor.c
        closest_pt = triangle(:,3);
    elseif edge.ab
        closest_pt = (eye(2) - line_ba * line_ba'/dot(line_ba, line_ba)) * triangle(:,2);
    elseif edge.bc
        closest_pt = (eye(2) - line_cb * line_cb'/dot(line_cb, line_cb)) * triangle(:,3);
    elseif edge.ca
        closest_pt = (eye(2) - line_ac * line_ac'/dot(line_ac, line_ac)) * triangle(:,1);
    else % inside the triangle
        closest_pt = [0; 0];
    end
           
end

%% Helper functions
% cross product wrapper function
function result = cross_wrap(a, b)
    arguments
        a (:,1);
        b (:,1);
    end
    a = [a; zeros(3 - length(a), 1)];
    b = [b; zeros(3 - length(b), 1)];
    
    result = cross(a, b);
end

% dot product wrapper function
function result = dot_wrap(a, b)
    arguments
        a (:,1);
        b (:,1);
    end
    
    n = max(length(a), length(b));
    a = [a; zeros(n - length(a))];
    b = [b; zeros(n - length(b))];
    
    result = dot(a, b);
end