%% needle_kinematics.m
%
% function to compute needle kinematic update given a control using unicycle
% model
%
% - written by: Dimitri Lezcano

function q_update = needle_kinematics(q, u, radius, dt)
    %% Arguments block
    arguments
        q (3,1); % [x; y; theta]
        u (2,1); % [u_phi; u_omega] 
        radius double;
        dt double;
    end
    
    %% Find the differential update
    % integrated form
    if u(2) ~= 0
        dq = [radius * u(1)/u(2) * ( sin(q(3) + dt) - sin(q(3)));
              radius * u(1)/u(2) * (-cos(q(3) + dt) + cos(q(3)));
              dt*u(2)]; 
    else
        dq = dt * [radius * u(1) * cos(q(3));
               radius * u(1) * sin(q(3));
               u(2)];
    end
    
    % Eulerian approximation
%     dq = dt * [radius * u(1) * cos(q(3));
%                radius * u(1) * sin(q(3));
%                u(2)];
    
    %% Update the current state
    q_update = q + dq;
    
end
    