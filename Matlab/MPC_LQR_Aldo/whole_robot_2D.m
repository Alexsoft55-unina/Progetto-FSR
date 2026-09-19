function ddq = whole_robot_2D(q, dq, tau)
    [M, CdQ, G] = calc_dynamics(q, dq);
    ddq = M \ (tau - CdQ - G);
end