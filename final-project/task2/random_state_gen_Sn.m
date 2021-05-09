%% random_state_gen_Sn.m
%
% function for creating random state generator for n-D torus
%
% - written by: Dimitri Lezcano

function qrand = random_state_gen_Sn(n)
    
    %% Arguments block
    arguments
        n {mustBeInteger};
    end
    
    qrand = 2*pi*rand(n,1); % (0, 2*pi)
end