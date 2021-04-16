%% test intersect line

%% Setup
p0 = [1;-2];
p1 = [2;2];
line = [p0, p1];

obs = [1 2 2 1; 0 0 1 1];
obs_idx = [1:size(obs,2), 1];


%% Intersect test
[does_intersect, tE, tL] = isintersect_linepolygon(obs, line);
if does_intersect
    disp('Intersects!');
    pE = p0 + tE * (p1 - p0)
    pL = p0 + tL * (p1 - p0)
else
    disp('Does not intersect!');
end


%% Plot 
plot(line(1,:), line(2,:)); hold on;
patch(obs(1,:), obs(2,:), 'yellow'); 

for i = 1:length(obs)
    j = obs_idx(i); k = obs_idx(i+1);
    e_i = obs(:,k) - obs(:,j);
    mp = mean(obs(:,[j,k]), 2);
    n_i = [0 1; -1 0] * e_i;
    
    line_i = mp + [[0;0] n_i];
    
    plot(line_i(1,:), line_i(2,:),'r');
end
xlim([-3,3]); ylim([-3,3]);
grid on;
hold off;
