%% task1.m
%
% main script for RMP n-link planar serial manipulator
%
% - written by: Dimitri Lezcano

function task1
    %% Set-up
    % robot parameters
    S.n = 4;
    S.link_len = 1;
    
    % initial and goal configurations
    qI = zeros(S.n,1);
    qG = [pi/2; -pi/3; -pi/3; pi/2.5];
    
    qI = Sn_bounds(qI);
    qG = Sn_bounds(qG);
    
    % obstacles
    obs = {[1/2, 1, 1, 1/2;
            1/2, 1/2, 1, 1]};
    obs{2} = -1.5*obs{1};
    obs{3} = [1, 1.5, 2, 2, 1
              2, 1.8, 2, 3, 3];
          
    S.obs = obs;
    
    % wraparound S
    wraparound = true;
    
    %% PRM and RRT set-up
    % random state generator
    rand_state_gen = @() random_state_gen_Sn(S.n);
    
    % discretization params
    S.dq = 0.1;
    
    % distance metric
    dist_fn = @(q, Q) dist_fn_knnsearch(q, Q, wraparound);
    
    % PRM parameters
    N_PRM = 500;
    K = 15;
    obstacle_dq = 0.1;
    
    % RRT parameters
    N_RRT = 3000;
    q_bound_fn = @(q) Sn_bounds(q, false);
    epsilon = 2;
    
    %% PRM method
    [path_prm, V_prm, E_prm, G_prm] = build_PRM(qI, qG, N_PRM, K, rand_state_gen, ...
        S, dist_fn, 'obstacle_dq', obstacle_dq, 'line_wraparound', wraparound);
    disp('PRM')
    disp(size(path_prm));
    
    % generate sequence
    movement_prm = zeros(4,0);
    for i = 1:size(path_prm, 2)-1
        line_i = generate_Sn_line(path_prm(:,i), path_prm(:,i+1), obstacle_dq, 'wraparound', wraparound);
        if i == 1
            movement_prm = [movement_prm, line_i];
        else
            movement_prm = [movement_prm, line_i(:,2:end)];
        end
    end 
    
    disp(' ');
    
    %% RRT Method
    [path_rrt, V_rrt, E_rrt, G_rrt] = build_RRT(qI, qG, N_RRT, S.dq/2, rand_state_gen, ...
        S, q_bound_fn, dist_fn, epsilon, 'wraparound', wraparound); 
    disp("RRT"); 
    disp(size(path_rrt));
    
    % generate sequence
    movement_rrt = zeros(4,0);
    for i = 1:size(path_rrt, 2)-1
        line_i = generate_Sn_line(path_rrt(:,i), path_rrt(:,i+1), obstacle_dq, 'wraparound', wraparound);
        if i == 1
            movement_rrt = [movement_rrt, line_i];
        else
            movement_rrt = [movement_rrt, line_i(:,2:end)];
        end
    end 
    
    disp("Press [ENTER] to continue.");
    pause;
    disp(' ');
    %% Plotting
    fig = figure(1);
    pI = plot_robot(qI, S.link_len, 'linespec', 'b-o'); hold on;
    pG = plot_robot(qG, S.link_len, 'linespec', 'g-o'); hold on;
    pO = plot_obstacles(obs, 'color', 'black'); hold on;
    grid on;
    xlabel('x'); ylabel('y');
    legend([pI, pG, pO], 'start', 'goal', 'obstacles', 'test');
    hold off;
    
    pause(1);
    %% Movies
    % PRM
    if ~isempty(path_prm)
        writematrix(movement_prm, 'prm_movement.csv')
        disp('Saved movement matrix: prm_movement.csv');
        fig_prm = figure(2);
        
        % video file writer
        vidfile = VideoWriter("task1_prm_results.mp4",'MPEG-4');
        vidfile.FrameRate = 5;
        open(vidfile);

        for i = 1:size(movement_prm, 2)
            q_i = movement_prm(:,i);
            
            % plotting
            fig_prm;
            pI = plot_robot(qI, S.link_len, 'linespec', 'b-o'); hold on;
            pG = plot_robot(qG, S.link_len, 'linespec', 'g-o'); hold on;
            pO = plot_obstacles(obs, 'color', 'black'); hold on;
            pcurrent = plot_robot(q_i, S.link_len, 'linespec', 'k-o'); hold off;
            grid on;
            xlabel('x'); ylabel('y');
            legend([pI, pG, pO, pcurrent], 'start', 'goal', 'obstacles', 'current');
            title("PRM Method");
            pause(0.1);
            
            % write the frame
            F = getframe(fig_prm);
            writeVideo(vidfile, F);
        end
        close(vidfile);
    end
    
    % RRT
    pause(1);
    if ~isempty(path_rrt)
        fig_rrt = figure(3);
        
        writematrix(movement_rrt, 'rrt_movement.csv')
        disp('Saved movement matrix: rrt_movement.csv');
        
        % video file writer
        vidfile = VideoWriter("task1_rrt_results.mp4",'MPEG-4');
        vidfile.FrameRate = 5;
        open(vidfile);
        
        for i = 1:size(movement_rrt, 2)
            q_i = movement_rrt(:,i);
            
            % plotting
            fig_rrt;
            pI = plot_robot(qI, S.link_len, 'linespec', 'b-o'); hold on;
            pG = plot_robot(qG, S.link_len, 'linespec', 'g-o'); hold on;
            pO = plot_obstacles(obs, 'color', 'black'); hold on;
            pcurrent = plot_robot(q_i, S.link_len, 'linespec', 'k-o'); hold off;
            grid on;
            xlabel('x'); ylabel('y');
            legend([pI, pG, pO, pcurrent], 'start', 'goal', 'obstacles', 'current');
            title('RRT Method');
            pause(0.1);
        
            % write the frame
            F = getframe(fig_rrt);
            writeVideo(vidfile, F);
        end
        close(vidfile);
    end
end

%% Helper functions
% PRM Helper
function dists = dist_fn_knnsearch(q, Q, wraparound)
    arguments
        q (:,1);
        Q (:,:);
        wraparound logical;
    end
    
    dists = zeros(size(Q,1), 1);
    for i = 1:size(Q, 1)
        q_i = Q(i,:)';
        
        dists(i) = metric_Sn(q, q_i, 'wraparound', wraparound);
    end
    
end

% helper plot functions    
function p = plot_robot(q, link_len, options)
    arguments
        q (:,1);
        link_len double;
        options.linespec = 'k-o';
        options.DisplayName = '';
    end
    ax_lim = link_len * numel(q);
    robot_joints = get_robot_joints(q, link_len);
    
    p = plot(robot_joints(1,:), robot_joints(2,:), options.linespec, ...
            'DisplayName', options.DisplayName);
    xlim([-ax_lim, ax_lim]); ylim([-ax_lim, ax_lim]);
end

function p = plot_obstacles(obs, options)
    arguments
        obs cell;
        options.color = 'red';
        options.DisplayName = 'obstacles'
    end
    
    for i = 1:numel(obs)
        obs_i = obs{i};
        
        p = patch(obs_i(1,:), obs_i(2,:), options.color, ...
                'Displayname', options.DisplayName);
    end
end  