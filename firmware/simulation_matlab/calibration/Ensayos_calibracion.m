clc; close all; clear;

% ================== CONFIGURACIÓN BÁSICA ==================
file       = 'Medidas motores DRON.xlsx';
sheetName  = 'MOTOR 1';      % <-- CAMBIÁ ESTO AL NOMBRE REAL DE LA HOJA
ExcelRange = 'H2:N23';       % Rango donde está la tabla promediada
g2N        = 9.80665e-3;     % 1 g_f = 9.80665e-3 N

% ================== LECTURA DE LA HOJA ÚNICA ==================
opts = detectImportOptions(file, ...
    'Sheet', sheetName, ...
    'Range', ExcelRange, ...
    'ReadVariableNames', true, ...
    'PreserveVariableNames', true);

opts = setvaropts(opts, opts.VariableNames, ...
    'TreatAsMissing', {'#DIV/0!','NA','-',''});

opts = setvartype(opts, opts.VariableNames, 'double');

T = readtable(file, opts);

% Nombres de columnas en minúscula para buscarlas por patrón
names = lower(T.Properties.VariableNames);
has   = @(pat) contains(names, pat);

% --- Columnas crudas ---
pwm        = T{:, has('pwm')};
omega_all  = T{:, has('velocidad') | has('omega') | contains(names,'ω')}; % rad/s (columna I)
rpm        = T{:, has('rpm')};
I          = T{:, has('intensidad') | has('corriente')};
vbat       = T{:, has('bateria') | has('v ') | strcmp(names,'v')};
Fg_all     = T{:, has('fuerza') | has('empuje')};      % en la hoja está en GRAMOS

motor.name = sheetName;

% ================== AJUSTE F[N] = b + Kt * (ω²) ==================
% Filtro: solo puntos donde haya ω y F
mF     = isfinite(omega_all) & isfinite(Fg_all);
omegaF = omega_all(mF);
FgF    = Fg_all(mF);

motor.omega_rad_s = omegaF(:);
motor.F_empuje_g  = FgF(:);
motor.F_empuje_N  = FgF(:) * g2N;   % convertir a Newtons

grado_F = 1;   % lineal en ω²
fprintf('\n==== Resultados (F = b + Kt * ω^2) para %s ====\n', motor.name);

X_F = motor.omega_rad_s.^2;   % ω^2
Y_F = motor.F_empuje_N;       % F en Newtons

[coef_F, fh_F, stats_F] = Minimos_cuadrados(X_F, Y_F, grado_F);
b_F  = coef_F(1);        % intercepto
Kt   = coef_F(2);        % pendiente -> N / (rad/s)^2

motor.fitFU_grado = grado_F;
motor.fitFU_coef  = coef_F;
motor.fitFU_R2    = stats_F.R2;
motor.fitFU_RMSE  = stats_F.RMSE;

fprintf('Kt = %.6e  [N·s^2/rad^2],   b = %.6e  [N],   R2 = %.4f,  RMSE = %.4g N\n', ...
        Kt, b_F, stats_F.R2, stats_F.RMSE);

% --------- Gráfico F vs ω² ---------
figure('Name','Empuje vs \omega^2');
scatter(X_F, Y_F, 30, 'filled'); hold on; grid on;

xx = linspace(min(X_F), max(X_F), 200);
plot(xx, fh_F(xx), 'LineWidth', 1.3);

xlabel('\omega^2 [rad^2/s^2]');
ylabel('F [N]');
title(sprintf('%s: F = b + K_t \\cdot \\omega^2', motor.name), 'Interpreter','none');

txt_F = sprintf('F = %.3e + %.3e \\cdot \\omega^2\nR^2 = %.4f', b_F, Kt, stats_F.R2);

xlim_ = xlim; 
ylim_ = ylim;
dx = diff(xlim_); 
dy = diff(ylim_);
xtext = xlim_(1) + 0.03*dx;
ytext = ylim_(2) - 0.10*dy;

text(xtext, ytext, txt_F, ...
    'FontSize', 10, ...
    'BackgroundColor',[1 1 1 0.5], ...
    'Margin',4);

% ================== AJUSTE PWM[µs] = a0 + a1·ω + a2·ω² ==================
fprintf('\n==== Ajuste PWM(ω) cuadrático para %s ====\n', motor.name);

% Filtro para relación PWM vs ω (NO usamos F acá)
mPWM       = isfinite(pwm) & isfinite(omega_all);
pwm_fit    = pwm(mPWM);
omega_fit  = omega_all(mPWM);   % rad/s

grado_wPWM = 2;   % cuadrático

% Ajuste PWM = a0 + a1*ω + a2*ω²
[coef_PWM, fh_PWM, stats_PWM] = Minimos_cuadrados(omega_fit, pwm_fit, grado_wPWM);

a0 = coef_PWM(1);
a1 = coef_PWM(2);
a2 = coef_PWM(3);

motor.fitPWMomega_grado = grado_wPWM;
motor.fitPWMomega_coef  = coef_PWM;
motor.fitPWMomega_R2    = stats_PWM.R2;
motor.fitPWMomega_RMSE  = stats_PWM.RMSE;

fprintf('PWM = a0 + a1·ω + a2·ω^2\n');
fprintf('a0 = %.6e  [PWM]\n',          a0);
fprintf('a1 = %.6e  [PWM / (rad/s)]\n', a1);
fprintf('a2 = %.6e  [PWM / (rad/s)^2]\n', a2);
fprintf('R2 = %.4f,  RMSE = %.4g PWM\n', stats_PWM.R2, stats_PWM.RMSE);

% --------- Gráfico PWM vs ω (cuadrático) ---------
figure('Name','PWM vs Velocidad angular');
scatter(omega_fit, pwm_fit, 30, 'filled'); hold on; grid on;

ww = linspace(min(omega_fit), max(omega_fit), 200);
plot(ww, fh_PWM(ww), 'LineWidth', 1.5);

xlabel('\omega [rad/s]');
ylabel('PWM [cuentas / \mus / % duty (según tu tabla)]');
title(sprintf('%s: Ajuste cuadrático PWM(\\omega)', motor.name), 'Interpreter','none');

txt_PWM = sprintf('PWM = %.3e + %.3e·\\omega + %.3e·\\omega^2\nR^2 = %.4f', ...
                  a0, a1, a2, stats_PWM.R2);

xlim_ = xlim; 
ylim_ = ylim;
dx = diff(xlim_); 
dy = diff(ylim_);
xtext = xlim_(1) + 0.03*dx;
ytext = ylim_(2) - 0.10*dy;

text(xtext, ytext, txt_PWM, ...
    'FontSize', 10, ...
    'BackgroundColor',[1 1 1 0.5], ...
    'Margin',4);

% ================== AJUSTE I[A] = c0 + c1*(ω) + c2*(ω)^2 ==================
fprintf('\n==== Ajuste I(ω^2) cuadrático para %s ====\n', motor.name);

% Filtro: puntos donde haya ω e I medidos
mI       = isfinite(omega_all) & isfinite(I);
omega_I  = omega_all(mI);      % rad/s
I_fit    = I(mI);              % A (según tu tabla)

grado_I = 2;   % cuadrático en ω^2

X_I = omega_I;   % variable independiente = ω^2
Y_I = I_fit;

[coef_I, fh_I, stats_I] = Minimos_cuadrados(X_I, Y_I, grado_I);

c0 = coef_I(1);
c1 = coef_I(2);
c2 = coef_I(3);

motor.fitIomega2_grado = grado_I;
motor.fitIomega2_coef  = coef_I;
motor.fitIomega2_R2    = stats_I.R2;
motor.fitIomega2_RMSE  = stats_I.RMSE;

fprintf('I = c0 + c1·(ω) + c2·(ω)^2\n');
fprintf('c0 = %.6e  [A]\n',              c0);
fprintf('c1 = %.6e  [A / (rad^2/s^2)]\n', c1);
fprintf('c2 = %.6e  [A / (rad^4/s^4)]\n', c2);
fprintf('R2 = %.4f,  RMSE = %.4g A\n',    stats_I.R2, stats_I.RMSE);

% --------- Gráfico I vs ω (cuadrático) ---------
figure('Name','Corriente vs \omega^2');
scatter(X_I, Y_I, 30, 'filled'); hold on; grid on;

xx_I = linspace(min(X_I), max(X_I), 200);
plot(xx_I, fh_I(xx_I), 'LineWidth', 1.5);

xlabel('\omega [rad/s]');
ylabel('I [A]');
title(sprintf('%s: Ajuste cuadrático I(\\omega)', motor.name), 'Interpreter','none');

txt_I = sprintf('I = %.3e + %.3e·(\\omega) + %.3e·(\\omega)^2\nR^2 = %.4f', ...
                c0, c1, c2, stats_I.R2);

xlim_ = xlim;
ylim_ = ylim;
dx = diff(xlim_);
dy = diff(ylim_);
xtext = xlim_(1) + 0.03*dx;
ytext = ylim_(2) - 0.10*dy;

text(xtext, ytext, txt_I, ...
    'FontSize', 10, ...
    'BackgroundColor',[1 1 1 0.5], ...
    'Margin', 4);

% Guardar coeficientes de I(ω) para el resumen final
I_c0   = c0;
I_c1   = c1;
I_c2   = c2;
I_R2   = stats_I.R2;
I_RMSE = stats_I.RMSE;


% ================== ENSAYO PAR ARRASTRE YAW: CÁLCULO DE Kq ==================
fprintf('\n==== Ensayo par arrastre Yaw: cálculo de Kq para %s ====\n', motor.name);

% --- Constante del motor ---
Kv_rpm_per_V = 1400;                   % [rpm/V] (ajustá si cambia tu motor)
Ke = 60 / (2*pi*Kv_rpm_per_V);         % [N·m/A]  (equivalente torque constant)

% --- Ajuste lineal I = I0 + c1 * ω^2 ---
mI      = isfinite(omega_all) & isfinite(I);
omega_I = omega_all(mI);               % [rad/s]
I_fit   = I(mI);                       % [A]

X_I = omega_I.^2;
Y_I = I_fit;
grado_I = 1;

[coef_I, fh_I, stats_I] = Minimos_cuadrados(X_I, Y_I, grado_I);

I0 = coef_I(1);                        % corriente de pérdidas
c1 = coef_I(2);                        % [A / (rad/s)^2]

% --- Pasar a torque: τ = Ke * I = τ0 + Kq * ω^2 ---
tau0 = Ke * I0;                        % [N·m]
Kq   = Ke * c1;                        % [N·m·s^2/rad^2]

motor.Ke        = Ke;
motor.I0_yaw    = I0;
motor.c1_Iw2    = c1;
motor.tau0_yaw  = tau0;
motor.Kq_yaw    = Kq;
motor.Iw2_R2    = stats_I.R2;
motor.Iw2_RMSE  = stats_I.RMSE;

fprintf('Ajuste I(ω^2): I = I0 + c1·ω^2\n');
fprintf('  I0  = %.6e [A]\n', I0);
fprintf('  c1  = %.6e [A/(rad/s)^2]\n', c1);
fprintf('  R2  = %.4f,  RMSE = %.4g A\n', stats_I.R2, stats_I.RMSE);

fprintf('\nConstante de torque equivalente:\n');
fprintf('  Kv  = %.1f [rpm/V]\n', Kv_rpm_per_V);
fprintf('  Ke  = %.6e [N·m/A]\n', Ke);

fprintf('\nModelo de par por arrastre (Yaw): τ = τ0 + Kq·ω^2\n');
fprintf('  τ0  = %.6e [N·m]\n', tau0);
fprintf('  Kq  = %.6e [N·m·s^2/rad^2]\n\n', Kq);


% --------- Gráfico torque de arrastre vs ω^2 (Kq) ---------
figure('Name','Par de arrastre yaw vs \omega^2');
% X_I y Y_I vienen del bloque de Kq: X_I = ω^2, Y_I = I
scatter(X_I, Ke*Y_I, 30, 'filled'); hold on; grid on;

xx_tau = linspace(min(X_I), max(X_I), 200);
plot(xx_tau, Ke*fh_I(xx_tau), 'LineWidth', 1.5);

xlabel('\omega^2 [rad^2/s^2]');
ylabel('\tau [N·m]');
title(sprintf('%s: Par de arrastre yaw, \\tau = \\tau_0 + K_q\\cdot\\omega^2', motor.name), ...
      'Interpreter','none');

txt_tau = sprintf('\\tau = %.3e + %.3e·\\omega^2\nK_q = %.3e [N·m·s^2/rad^2]', ...
                  tau0, Kq, Kq);

xlim_ = xlim;
ylim_ = ylim;
dx = diff(xlim_);
dy = diff(ylim_);
xtext = xlim_(1) + 0.03*dx;
ytext = ylim_(2) - 0.10*dy;

text(xtext, ytext, txt_tau, ...
    'FontSize', 10, ...
    'BackgroundColor',[1 1 1 0.5], ...
    'Margin',4);

% ================== RESUMEN FINAL DE PARÁMETROS ==================
fprintf('================= RESUMEN FINAL MOTOR  =================\n');

% Kt de F vs ω^2
fprintf('Empuje:   F(ω^2) = %.3e + %.3e·ω^2   [N]\n', b_F, Kt);

% Ajuste cuadrático PWM(ω)
fprintf('PWM(ω):   PWM(ω) = %.3e + %.3e·ω + %.3e·ω^2\n', a0, a1, a2);

% Ajuste cuadrático I(ω)
fprintf('Corriente: I(ω)  = %.3e + %.3e·ω + %.3e·ω^2   [A]\n', I_c0, I_c1, I_c2);

% Par de arrastre yaw
fprintf('Yaw:      τ(ω^2) = %.3e + %.3e·ω^2   [N·m]   =>   Kq = %.3e [N·m·s^2/rad^2]\n', ...
        tau0, Kq, Kq);

fprintf('==========================================================\n');
