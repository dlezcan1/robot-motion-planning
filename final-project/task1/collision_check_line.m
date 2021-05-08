% sample points along a line (in C-Space) to check for a collision
function colliding = collision_check_line(q1, q2, S, dq, options)
    arguments
        q1 (:,1);
        q2 (:,1);
        S struct;
        dq double = 0.1;
        options.wraparound logical = true;
    end
    
    % generate the sample q's along this line
    q_checks = generate_Sn_line(q1, q2, dq, 'wraparound', options.wraparound);
    
    % initialization
    colliding = false;
    
    % iterate through all of the sample q' forwards
    for i = 1:size(q_checks, 2)
        % check for collision
        if collision_detection(q_checks(:,i), S)
            colliding = true;
            break;
        end
    end
end