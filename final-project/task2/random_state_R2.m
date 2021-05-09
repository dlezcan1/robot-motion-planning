%% random_state_R2
%
% function for random generating functions for R2
% 
% - written by: Dimitri Lezcano

function qrand = random_state_R2(xmax, ymax, xmin, ymin)
    arguments
        xmax double;
        ymax double;
        xmin double = 0;
        ymin double = 0;
    end
    
    qrand = [xmax-xmin; ymax-ymin] .* rand(2,1) + [xmin; ymin];
end   