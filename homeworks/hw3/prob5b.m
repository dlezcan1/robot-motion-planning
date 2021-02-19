%% prob5b.m
%
% this is a script to draw the face with nose patch
%
% - written by: Dimitri Lezcano

%% Set-Up
% script options
save_bool = true;

points = [-2 2 0; -1 -1 2];

%% Plotting
% parametrization
t = linspace(0, 2*pi, 200);

fig = figure(1);
% configuration patch
patch(points(1,:), points(2,:), 'red'); hold on

% the other face components
plot(10*cos(t), 10*sin(t), 'b-'); hold on;
plot(cos(t) + 3, sin(t) + 6, 'k-'); hold on;
plot(cos(t) - 3, sin(t) + 6, 'k-'); hold on;
plot(4*cos(t), sin(t) - 7, 'k-'); hold on;
hold off; axis square


%% Saving
if save_bool
    saveas(fig, 'prob5b.png');
    disp('Saved figure: prob5b.png');
end