%% FUNZIONE MINIMUM JERK
function [pos, vel] = genera_profilo_quinto_ordine(t, target, T_manovra)
    if t <= 0
        pos = 0; vel = 0;
    elseif t >= T_manovra
        pos = target; vel = 0;
    else
        tau = t / T_manovra;
        pos = target * (10*tau^3 - 15*tau^4 + 6*tau^5);
        vel = (target / T_manovra) * (30*tau^2 - 60*tau^3 + 30*tau^4);
    end
end