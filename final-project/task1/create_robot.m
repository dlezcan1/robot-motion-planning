%% create_robot.m
%
% function to create robot linesegments
%
% - written by: Dimitri Lezcano

function robot_linesegs = create_robot(q,link_len)
    arguments
        q (:,1);
        link_len double;
    end
    % set-up
    e1 = [1;0];
    pt_base = zeros(2,1);
    robot_linesegs = zeros(2,2,length(q));
    
    % iterate through the arclengths
    R_i = eye(2);
    for i = 1:length(q)
        q_i = q(i);
        R_i = R_i*rot2d(q_i);
        lineseg = [pt_base, pt_base + link_len*R_i*e1];
        
        % update
        robot_linesegs(:,:,i) = lineseg;
        pt_base = lineseg(:,2);
    end
end