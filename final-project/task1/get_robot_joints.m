%% get_robot_joints.m
%
% function to get robot joint positions
%
% - written by: Dimitri Lezcano

function robot_joint_pts = get_robot_joints(q, link_len)
    arguments 
        q (:,1);
        link_len double;
    end
    
    robot = create_robot(q, link_len);
    robot_joint_pts = [squeeze(robot(:,1,:)), robot(:,2,end)];
    
end