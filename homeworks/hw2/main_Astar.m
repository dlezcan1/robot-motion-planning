%% main_Astar.m
%
% this is a main script to check the A* algorithm
%
% - written by: Dimitri Lezcano

%% Set-Up 
% Adjacency matrix
A = inf * ones(15, 15);
A(1:size(A, 1) + 1:end) = 0; % set diagonal elements to 0

A(1, [2, 6, 9]) = [3, 2, 4];
A(2, [3, 6, 8]) = [1, 1, 4];
A(3, [4]) = [3];
A(4, [5, 8, 12]) = [2, 1, 2];
A(5, [12, 15]) = [8, 3];
A(6, [7, 9]) = [1, 1];
A(7, [8, 10, 11]) = [3, 1, 1];
A(8, [11]) = [4];
A(9, [10, 13]) = [3, 5];
A(10, [11]) = [2];
A(11, [12, 13]) = [1, 3];
A(12, [14, 15]) = [4, 2];
A(13, [14]) = [6];

A = min(A, A'); % make sure it's undirected


%% Run Astar
disp('A*: number of nodes heuristic');
tic
path_num_nodes = Astar(A, 2, 14, 'heuristic', 'num_nodes')
toc

disp(' ');

disp('A*: row sum heuristic');
tic 
path_row_sum = Astar(A, 2, 14, 'heuristic', 'row_sum')
toc
