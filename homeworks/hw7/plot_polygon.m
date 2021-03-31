function plot_polygon(P)
    
    plt_idxs = [1:size(P, 2), 1];
    plot(P(1, plt_idxs), P(2, plt_idxs))
    
end