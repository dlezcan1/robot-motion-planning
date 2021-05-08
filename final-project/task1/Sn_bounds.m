function q_bound = Sn_bounds(q, mpi_to_pi)
    arguments
        q (:,:);
        mpi_to_pi logical = false;
    end
    % determine bounds
    if mpi_to_pi
        q_lo = -pi;
        q_hi = pi;
    else
        q_lo = 0;
        q_hi = 2*pi;
    end
    
    q_bound = q;
    
    % modulo up q_bound until between [q_lo, q_hi)
    while any(q_bound < q_lo, 'all')
        q_bound(q_bound < q_lo) = q_bound(q_bound < q_lo) + 2*pi;
    end
    
    % modulo down q_bound until between [q_lo, q_hi)
    while any(q_bound >= q_hi, 'all')
        q_bound(q_bound >= q_hi) = q_bound(q_bound >= q_hi) - 2*pi;
    end
    
end