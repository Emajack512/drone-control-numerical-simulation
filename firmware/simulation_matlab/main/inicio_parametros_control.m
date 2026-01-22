function parametros_control = inicio_parametros_control()
% Cargar AQUÍ las Kp/Ki/Kd EXACTAS de tu .cpp
% Unidades SI (rad, rad/s, N·m). Límites y antiwindup iguales a tu firmware.

% ---- Lazo externo: ángulo → tasa ref ----
    
%ROLL
    params.roll_PI.Kp  = 3.6642;  % [rad -> rad/s]
    params.roll_PI.Ki  = 0.0060;
    params.roll_PI.u_min = -4.0;
    params.roll_PI.u_max =  4.0;
    params.roll_PI.I_min = -1.0;  
    params.roll_PI.I_max = 1.0; 

%Pitch

params.pitch_PI.Kp  = 7.4330;  % [rad -> rad/s]
params.pitch_PI.Ki  = 0.0153;
params.pitch_PI.u_min = -4.0;
params.pitch_PI.u_max =  4.0;
params.pitch_PI.I_min = -1.0; 
params.pitch_PI.I_max = 1.0; 

% ---- Lazo interno: tasa → aceleración angular deseada ----

% VELOCIDAD P
params.p_PID.Kp   = 75.6397;   % [rad/s -> rad/s^2]
params.p_PID.Ki   = 5.1850;
params.p_PID.Kd   = 0.5932;
params.p_PID.u_min = -8.0;
params.p_PID.u_max =  8.0;
params.p_PID.I_min = -2.0; 
params.p_PID.I_max = 2.0; 

% VELOCIDAD Q
params.q_PID.Kp   = 21.1333;   % [rad/s -> rad/s^2]
params.q_PID.Ki   = 3.9098;
params.q_PID.Kd   = 0.2328;
params.q_PID.u_min = -8.0;
params.q_PID.u_max =  8.0;
params.q_PID.I_min = -2.0; 
params.q_PID.I_max = 2.0;

% VELOCIDAD R
params.r_PID.Kp   = 5.8820;   % [rad/s -> rad/s^2]
params.r_PID.Ki   = 6.4370;
params.r_PID.Kd   = 0.17;
params.r_PID.u_min = -3.0;
params.r_PID.u_max =  3.0;
params.r_PID.I_min = -1.5;
params.r_PID.I_max = 1.5;

% ---- Límites físicos de torques ----
params.tau_lim = [0.20; 0.20; 0.12]; % [N·m] (x,y,z)


% ---- Límites de empuje ----
params.U1_min = 0.0;    % [N]
params.U1_max = 30;    % [N]

% ---- Estados internos (integradores / filtros) ----
params.int.roll = 0.0; 
params.int.pitch = 0.0;
params.int.p = 0.0;  
params.int.q = 0.0;  
params.int.r = 0.0;

params.deriv_filter.p.prev = 0.0;
params.deriv_filter.q.prev = 0.0;
params.deriv_filter.r.prev = 0.0;

%Memoria de errores
params.err_prev.p = 0.0; params.err_prev.q = 0.0; params.err_prev.r = 0.0;


parametros_control = params;
end
