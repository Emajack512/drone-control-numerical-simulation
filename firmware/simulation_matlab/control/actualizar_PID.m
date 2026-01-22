function [u, I_out, ep_prev_out] = actualizar_PID(cfg, err, I_in, ep_prev_in, h)
% cfg: Kp, Ki, Kd, u_min, u_max, I_min, I_max
% Emula el patrón exacto del .cpp (derivada backward, clamp de I y undo en saturación).

% 1) Derivada backward
D = cfg.Kd * (err - ep_prev_in) / h;

% 2) Integrar y CLAMP del integrador
I = I_in + cfg.Ki * err * h;
I = min(max(I, cfg.I_min), cfg.I_max);

% 3) Salida base y saturación
u_base = cfg.Kp*err + I + D;
u = min(max(u_base, cfg.u_min), cfg.u_max);

% 4) Anti-windup "undo integration"
sat_hi = (u_base > cfg.u_max);
sat_lo = (u_base < cfg.u_min);
if ( (sat_hi && err > 0) || (sat_lo && err < 0) )
    I = I - cfg.Ki * err * h;                 % deshago la integración de este tick
    I = min(max(I, cfg.I_min), cfg.I_max);    % re-clamp
    u_base = cfg.Kp*err + I + D;
    u = min(max(u_base, cfg.u_min), cfg.u_max);
end

I_out = I;
ep_prev_out = err;
end
