%% test_script.m
%
% this is a script to test the ClosestPointOnTriangleToPoint, 
% ClosestPointOnTriangleToOrigin, and GJKalg_2D functions
%
% - written by: Dimitri Lezcano

%% Set-up
% closest point to triangles
triangle1 = [1 2 3;
            0 1 2];
triangle2 = [-1 0 1;
             -1 1 -1];
test_pt = [-1; 1/2];

         
%% Plot triangles and points
%% ClosestPointonTriangleToOrigin Test
disp('ClosestPointonTriangleToOrigin Test');

disp('Triangle 1');
[c_pt1, vor1, e1] = ClosestPointOnTriangleToOrigin(triangle1)

disp('Triangle 2');
[c_pt2, vor2, e2] = ClosestPointOnTriangleToOrigin(triangle2)

%% ClosestPointonTriangleToPoint Test
disp('ClosestPointonTriangleToPoint Test');

disp('Triangle 1');


disp('Triangle 2');

%% GJKalg_2D test
disp('GJKald_2d Test');