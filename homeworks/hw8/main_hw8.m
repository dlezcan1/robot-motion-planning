%% main_hw8
% 
% - written by: Dimitri Lezcano

%% Set-up
% initial and goal configurations
qI = [0;0];
qG = [3;3];

% obstacles
obs{1} = [1 2 2 1; 
          1 1 2 2];
      
% RRT Parameters
dq = 0.1;
epsilon = 0.5;
xmax = 5; ymax = xmax;
N_RRT = 500;

% PRM Parameters
K = 10;
N_PRM = 100;


%% RRT and PRM methods
% RRT
[path_rrt, V_rrt, E_rrt, G_rrt] = build_RRT(qI, qG, N_RRT, dq, obs, xmax, ymax,...
                                            epsilon);
E_rrt = full(G_rrt.adjacency);
if ~isempty(path_rrt)
    disp("RRT Success");
else
    disp("RRT Failure");
end

% PRM
[path_prm, V_prm, E_prm, G_prm] = build_PRM(qI, qG, N_PRM, K, obs, xmax, ymax);

if ~isempty(path_prm)
    disp("PRM Success");
else
    disp("PRM Failure");
end


%% Plotting
% RRT
figure(1);
pI = plot(qI(1), qI(2), 'r*'); hold on;
pG = plot(qG(1), qG(2), 'g*'); hold on;
for i = 1:numel(obs)
    obs_i = obs{i};
    pO = patch(obs_i(1,:), obs_i(2,:),'black'); hold on;
end

% - Plot the graph
for i=1:size(E_rrt,2)
    for j=1:size(E_rrt,2)
        if E_rrt(i,j)~=inf && E_rrt(i,j)~=0
            pGr = plot([V_rrt(1,i),V_rrt(1,j)],[V_rrt(2,i),V_rrt(2,j)],'black');
        end
    end
end

% - Plot the Path
if ~isempty(path_rrt)
    p_path = plot(path_rrt(1,:), path_rrt(2,:), 'b-');
else
    p_path = [];
end

% - plot config
legend([pI, pG, pO, pGr, p_path], 'start', 'goal', 'obstacles', 'graph', 'path');
xlim([0, xmax]); ylim([0, ymax]);
grid on; title('RRT Method');
xlabel('x'); ylabel('y');
hold off;


% PRM
figure(2);
pI = plot(qI(1), qI(2), 'r*'); hold on;
pG = plot(qG(1), qG(2), 'g*'); hold on;
for i = 1:numel(obs)
    obs_i = obs{i};
    pO = patch(obs_i(1,:), obs_i(2,:),'black');
end

% - Plot the graph
for i=1:size(E_prm,2)
    for j=1:size(E_prm,2)
        if E_prm(i,j)~=inf && E_prm(i,j)~=0
            pGr = plot([V_prm(1,i),V_prm(1,j)],[V_prm(2,i),V_prm(2,j)],'black');
        end
    end
end

% - Plot the Path
if ~isempty(path_prm)
    p_path = plot(path_prm(1,:), path_prm(2,:), 'b-');
else
    p_path = [];
end

% - plot config
legend([pI, pG, pO, pGr, p_path], 'start', 'goal', 'obstacles', 'graph', 'path');
xlim([0, xmax]); ylim([0, ymax]);
grid on; title('PRM Method');
xlabel('x'); ylabel('y');
hold off;



