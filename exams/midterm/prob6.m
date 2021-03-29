%% prob6.m
%
% - written by: Dimitri Lezcano

%% Set-Up
% mesh grid
dl = 0.1;
x = -1:dl:9;
y = -5:dl:5;
[X, Y] = meshgrid(x,y);

%% part a
f0_mask = robot_f(X, Y);
f0 = double(f0_mask)*2;
g_mask = obs_g(X, Y);
g = double(g_mask);

fig1 = figure(1);
surf(X, Y, f0, 'DisplayName', "f(x)", 'EdgeColor', 'none'); hold on;
surf(X, Y, g, 'DisplayName', "g(x)",'EdgeColor', 'none'); 
hold off;
view(2);colormap('gray');
xlabel('x'); ylabel("y");
title("Problem 6 Part a")

%% part b
% find convolution
Cobs_conv = conv2(f0, g)*dl;
[X2, Y2] = meshgrid(-1:dl/2:9, -5:dl/2:5);
X2 = 2*X2; Y2 = 2*Y2;

fig2 = figure(2);
surf(X2, Y2, Cobs_conv, 'EdgeColor', 'none');
xlim([-1,9]); ylim([-5,5]);
title("Problem 6 part b");
view(2);

%% part c
fig3 = figure(3);
surf(X2, Y2, double(Cobs_conv > 0), 'EdgeColor', 'none');
xlim([-1,9]); ylim([-5,5]); colormap('gray')
title("Problem 6 part c");
view(2);


%% part d
fig4 = figure(4);
t = linspace(0, 2*pi, 200);
Cobs_bnd_thy = [3*cos(t) + 3;
             2*sin(t)];
plot(Cobs_bnd_thy(1,:), Cobs_bnd_thy(2,:));
xlim([-1,9]); ylim([-5,5]);
title('Problem 6 Part d');

%% SAving
saveas(fig1, 'parta.png');
saveas(fig2, 'partb.png');
saveas(fig3, 'partc.png');
saveas(fig4, 'partd.png');


%% helper functions
function Z = robot_f(X, Y)
    Z = (X.^2 + Y.^2) <= 1;
    
end

function Z = obs_g(X, Y)
    Z = ((X-3).^2/4 + Y.^2) <= 1;
    
end