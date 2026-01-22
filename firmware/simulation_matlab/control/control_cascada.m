function [U, params,mon] = control_cascada(refs, X, phys, params, h)
% Entradas:
%  refs: .roll_ref [rad], .pitch_ref [rad], .r_ref [rad/s], .throttle [-1,1]
%  X   : [x y z vx vy vz phi th psi p q r]'
%  phys: .m, .g, .I
%  params: ver init_params_control
% Salida: U = [U1; tau_x; tau_y; tau_z]

phi = X(7); th = X(8);
p = X(10); q = X(11); r = X(12);

% --- (a) PI externo: ángulo → tasa ref ---
e_phi = wrapAngle(refs.roll_ref  - phi);
e_th  = wrapAngle(refs.pitch_ref - th);

[p_ref, params.int.roll]  = actualizar_PI(params.roll_PI,  e_phi, params.int.roll,  h);
[q_ref, params.int.pitch] = actualizar_PI(params.pitch_PI, e_th,  params.int.pitch, h);


% --- (b) PID interno: tasa → alpha_des ---
err_p = p_ref      - p;
err_q = q_ref      - q;
err_r = refs.r_ref - r;

[alpha_p, params.int.p, params.err_prev.p] = actualizar_PID(params.p_PID, err_p, params.int.p, params.err_prev.p, h);
[alpha_q, params.int.q, params.err_prev.q] = actualizar_PID(params.q_PID, err_q, params.int.q, params.err_prev.q, h);
[alpha_r, params.int.r, params.err_prev.r] = actualizar_PID(params.r_PID, err_r, params.int.r, params.err_prev.r, h);


alpha_des = [alpha_p; alpha_q; alpha_r];

% --- (c) Torques físicos: τ = I α + ω×(Iω) ---
I = phys.I; w = [p;q;r];
tau_unsat = I*alpha_des + cross(w, I*w);
%Saturacion
tau = [ saturar(tau_unsat(1), -params.tau_lim(1), params.tau_lim(1))
        saturar(tau_unsat(2), -params.tau_lim(2), params.tau_lim(2))
        saturar(tau_unsat(3), -params.tau_lim(3), params.tau_lim(3)) ];

% --- (d) Empuje desde throttle (sin compensación de inclinación) ---
U_hover = phys.m*phys.g;
K_U1    = 0.1;
U1 = U_hover + K_U1 * refs.throttle*phys.m*phys.g;
U1 = saturar(U1, params.U1_min, params.U1_max);

U = [U1; tau];
mon.p_ref     = p_ref;
mon.q_ref     = q_ref;
mon.alpha_des = alpha_des;
mon.tau_unsat = tau_unsat;
mon.R33       = cos(phi)*cos(th);  % por si querés ver inclinación efectiva

end
