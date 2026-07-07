function q_des_calc = calcola_target_cinematici(h_des, robot_params,q0_guess)
    % Questa funzione viene eseguita al 100% nel motore MATLAB puro,
    % quindi non ha alcuna limitazione del Simulink Coder.
    
    cinematica_vincoli = @(q_var) [
        (robot_params.m1*robot_params.lc1 + (robot_params.m2+robot_params.m3)*robot_params.l1)*cos(q_var(1)) + ...
        (robot_params.m2*robot_params.lc2 + robot_params.m3*robot_params.l2)*cos(q_var(1)+q_var(2)) + ...
        (robot_params.m3*robot_params.lc3)*cos(q_var(1)+q_var(2)+q_var(3));
        
        q_var(1) + q_var(2) + q_var(3) - (pi/2);
        
        robot_params.l1*sin(q_var(1)) + robot_params.l2*sin(q_var(1)+q_var(2)) + robot_params.l3*sin(q_var(1)+q_var(2)+q_var(3)) - h_des
    ];

    options = optimoptions('fsolve', 'Display', 'off');
    
    
    q_des_calc = fsolve(cinematica_vincoli, q0_guess, options);
end