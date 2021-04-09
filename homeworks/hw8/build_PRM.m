%% build_PRM.m
%
% function to build PRM.
%
% Args:
%   - qI and qG: initial and goal position of the robot.
%   – NumNodes: the limit number of nodes.
%   - K: the number of nearest neighbors
%   - obstacles: cell array of 2xN matrices (default is {})
%   - xmax, ymax: Positive doubles representing C-space bounds (default = 1,
%                   xmax)
%
% Return:
%   – the path connecting qI and qG.
%   – The set of vertices V and the set of edges E
%   - The MATLAB graph representation of the built PRM


function [path, V, E, G] = build_PRM(qI , qG, NumNodes, K, obstacles, xmax, ymax)
    %% Arguments Block
    arguments
        qI (2,1);
        qG (2,1);
        NumNodes double {mustBeInteger};
        K double {mustBeInteger};
        obstacles cell = {};
        xmax double {mustBePositive} = 1;
        ymax double {mustBePositive} = xmax;
    end
    
    %% 

end