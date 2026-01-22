function [u, I_out] = actualizar_PI(cfg, err, I_in, h)
% cfg: Kp, Ki, u_min, u_max, I_min, I_max
% Emula: I += Ki*e*Ts (clamp); u_base = Kp*e + I; u = sat(u_base);
% Anti-windup tipo "undo integration" si satura y el error empuja hacia afuera.

% 1) Integrar y CLAMP del integrador (igual que I_roll_min/max)
I = I_in + cfg.Ki*err*h;
I = min(max(I, cfg.I_min), cfg.I_max);

% 2) Salida base y saturación
u_base = cfg.Kp*err + I;
u = min(max(u_base, cfg.u_min), cfg.u_max);

% 3) Anti-windup "undo": si saturó y el error empuja hacia afuera, deshago la integración
sat_hi = (u_base > cfg.u_max);
sat_lo = (u_base < cfg.u_min);
if ( (sat_hi && err > 0) || (sat_lo && err < 0) )
    I = I - cfg.Ki*err*h;             % quitamos lo que acabamos de sumar
    I = min(max(I, cfg.I_min), cfg.I_max);  % re-clamp por las dudas
    u_base = cfg.Kp*err + I;
    u = min(max(u_base, cfg.u_min), cfg.u_max);
end

I_out = I;
end
