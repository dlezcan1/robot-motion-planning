%% task2.m
%
% task 2's main file for planar needle RRT using unicycle model
%
% - written by: Dimitri Lezcano

function task2
    %% Set-up
    % needle parameters
    radius = 0.5; % radius of curvature
    
    % needle possible controls
    needle_controls = {0;
                       -radius;
                       radius};
    
    % initial and goal configurations
    qI = [0;0;0];
    qG = [20;-10;0];
    
    % obstacles
    obstacles = {};
    
    % RRT parameters
    N = 6000;
    epsilon = 0.5;
    dt = 0.5;
    dl = 10;
    xmax = max(qI(1), qG(1)) + dl;
    ymax = max(qI(2), qG(2)) + dl;
    xmin = min(qI(1), qG(1)) - dl;
    ymin = min(qI(2), qG(2)) - dl;
    
    % RRT Funcitions
    kinematics = @(q,u,dt) needle_kinematics(q, [1;u], radius, dt);
    collision_check = @ needletip_collision_detection;
    random_state_gen = @() [random_state_R2(xmax, ymax, xmin, ymin),
                            random_state_gen_Sn(1)];
    dist_metric = @(x,X) needle_tip_dist(x, X);
    
    % Plot options
    N_traj = 10;
                        
    %% RRT method
    [path, V, E, G] = build_kin_RRT(qI, qG, N, dt, random_state_gen, kinematics,...
                                    needle_controls, epsilon, obstacles, collision_check, ...
                                    dist_metric);
    if ~isempty(path)
        disp("RRT: Success!");
    else
        disp("RRT: Failure :(");
    end
    
    %% Plotting
    fig = figure(1);
    
    pI = plot(qI(1), qI(2), 'r*'); hold on;
    pG = plot(qG(1), qG(2), 'g*'); hold on;
    pO = [];
    for i = 1:numel(obstacles)
        obs_i = obstacles{i}; 
        pO = patch(obs_i(1,:), obs_i(2,:), 'black'); hold on;
    end
    
    % Plot the graph
%     for i=1:size(E,2)
%         for j=1:size(E,2)
%             if E(i,j)~=inf && E(i,j)~=0
%                 pGr = plot([V(1,i),V(1,j)],[V(2,i),V(2,j)],'black'); hold on;
%             end
%         end
%     end
    
    for edge_i = 1:G.numedges
        i = G.Edges.EndNodes(edge_i, 1);
        j = G.Edges.EndNodes(edge_i, 2);
        u_ij = G.Edges.controls(edge_i,:);
        traj_ij = V(:,[i,j]);
        
        pGr = plot(traj_ij(1,:), traj_ij(2,:), 'black'); hold on;
    end
        

    % Plot the Path
    if ~isempty(path)
        p_path = plot(path(1,:), path(2,:), 'b-'); hold on;
    else
        p_path = [];
    end
    
    % configure plot
    if isempty(obstacles)
        legend([pI, pG, pGr, p_path], 'start', 'goal', 'graph', 'path');
    else
        legend([pI, pG, pO, pGr, p_path], 'start', 'goal', 'obstacles', 'graph', 'path');
    end
    xlim([xmin, xmax]); ylim([ymin, ymax]);
    axis equal; grid on; 
    title("Needle RRT");
    xlabel('x'); ylabel('y');
    hold off;
    
    %% Saving
    if ~isempty(path)
        saveas(fig, 'rrt_results.png');
    end
    
end

%% Helper functions
function curv_path = generate_trajectory(q, u, dt, N, kinematics)
    curv_path = [q];
    for i = 1:N
        q_i = kinematics(curv_path(:,end), u, dt);
        curv_path = [curv_path, q_i];
    end
end
