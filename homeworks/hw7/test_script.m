%% test_script.m
%
% this is a script to test the ClosestPointOnTriangleToPoint, 
% ClosestPointOnTriangleToOrigin, and GJKalg_2D functions
%
% - written by: Dimitri Lezcano

%% Set-up
% triangle set-up
triangle1 = [1 2 3;
             0 1 0];
triangle2 = [-1 0 1;
             -1 1 0];
p_idxs = [1:3, 1];

test_pt = [1/2; -2];
         
figure(1);
subplot(1,2,1);
plot(triangle1(1, p_idxs), triangle1(2,p_idxs)); hold on;
plot(0, 0, 'k*'); 
plot(test_pt(1), test_pt(2), 'g*'); hold off;
axis square; grid on;
title('triangle 1')

subplot(1, 2, 2);
plot(triangle2(1, p_idxs), triangle2(2,p_idxs)); hold on;
plot(0, 0, 'k*'); 
plot(test_pt(1), test_pt(2), 'g*'); hold off;
axis square; grid on;
title('triangle 2')


%% ClosestPointonTriangleToOrigin Test
disp('ClosestPointonTriangleToOrigin Test');

disp('Triangle 1');
[c_pt1, vor1, e1] = ClosestPointOnTriangleToOrigin(triangle1)

disp('Triangle 2');
[c_pt2, vor2, e2] = ClosestPointOnTriangleToOrigin(triangle2)

input('Press [ENTER] to continue');

disp(' ');

%% ClosestPointonTriangleToPoint Test
disp('ClosestPointonTriangleToOrigin Test');

disp('Triangle 1');
[c_pt1, vor1, e1] = ClosestPointOnTriangleToPoint(triangle1, test_pt)

disp('Triangle 2');
[c_pt2, vor2, e2] = ClosestPointOnTriangleToPoint(triangle2, test_pt)

input('Press [ENTER] to continue');

disp(' ');

%% GJKalg_2D test
disp('ClosestPointonTriangleToOrigin Test');


disp('Program completed.');
