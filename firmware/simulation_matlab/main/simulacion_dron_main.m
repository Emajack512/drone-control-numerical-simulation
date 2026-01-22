function simulacion_dron_main

clc; clear all; close all;

% --- Tema oscuro global (figuras y ejes) ---
bg = [0.08 0.08 0.10];   % casi negro
fg = [1 1 1];            % texto blanco

set(groot, ...
  'DefaultFigureColor',      bg, ...
  'DefaultAxesColor',        bg, ...
  'DefaultAxesXColor',       fg, ...
  'DefaultAxesYColor',       fg, ...
  'DefaultAxesZColor',       fg, ...
  'DefaultAxesGridColor',    [0.40 0.40 0.40], ...
  'DefaultAxesGridAlpha',    0.25, ...
  'DefaultAxesLineWidth',    0.9, ...
  'DefaultLineLineWidth',    1.6, ...
  'DefaultTextColor',        fg, ...
  'DefaultLegendTextColor',  fg, ...
  'DefaultLegendBox',        'off' ...
);

% ================== Parametros fisicos ==========================

m = 0.953; % masa [kg]
g = 9.81; %gravedad [m/s^2]
I = [ 0.007853      ,  8.3306e-07 ,  4.0056681e-05;   % I_xx, I_xy, I_xz
      8.3306e-07   ,  0.008419   , -9.40363e-07;      % I_yx, I_yy, I_yz
      4.0056681e-05, -9.40363e-07,  0.01538      ];   % I_zx, I_zy, I_zz

l = 0.21379; %distancia de brazo del centro del motor al centro de masa [metros]

% ================= Constantes de relaciones =======================

Kt=1.551e-05; %relacion entre el empuje total y la velocidad angular de las helices
%S=[1,-1,1,-1]; %secuencia de par de arrastre del sentido de giro 
Kq=2.061e-07; %relacion lineal entre el Tz y la velocidad angular de las helices

% ===== PWM por motor: uS = a + b*w + c*w^2 =====
pwm_poly = [ 1.041e+03, 8.145e-02, 2.190e-03;   % m1: [a b c]
             1.041e+03, 8.145e-02, 2.190e-03;   % m2
             1.041e+03, 8.145e-02, 2.190e-03;   % m3
             1.041e+03, 8.145e-02, 2.190e-03 ]; % m4
us_min = 1000;  
us_max = 2000;      % límites físicos


phys.m = m; phys.g = g; phys.I = I;  %Empaquetamos los parameteros

corriente_cuadratica = [ 6.445e-01, -1.541e-02, 5.210e-05;   % m1: [aI bI cI]
                         6.445e-01, -1.541e-02, 5.210e-05;   % m2
                         6.445e-01, -1.541e-02, 5.210e-05;   % m3
                         6.445e-01, -1.541e-02, 5.210e-05 ]; % m4

I_max_motor = 10.0;     % [A] límite por motor
V_pack      = 11.1;     % [V] (ej: 3S nominal)
enforce_Imax = true;    % activar límite de corriente por motor

parametros_ctrl = inicio_parametros_control();       % ACA SE ENCUENTRAN LAS CONSTANTES DEL PI/PID

% Referencias iniciales (hover)
refs.roll_ref  = 0.0;   % [rad]   [-15°,15]
refs.pitch_ref = 0.0;   % [rad]   [-15°,15]
refs.r_ref     = 0.0;   % [rad/s] [-1,1]
refs.throttle  = 0.0;   % [-1,1]

% =============== Parametros temporales RK4 =======================

h = 0.01; %Paso por RK4
T_total = input('Ingrese el tiempo total de la simulacion: '); % Duracion total de la simulacion
N = round(T_total/h); %Cantidad de pasos de la simulacion, redondeada
t = (0:N-1)*h; %Vector de tiempos para graficar

% ============= Parametros iniciales (Estado inicial del dron) ==================

% Vector de paramteros iniciales
X = [0; 0; 0;    % posición inicial en el mundo (x, y, z) [metros]
     0; 0; 0;    % velocidad inicial en el mundo (vx, vy, vz) [m/s]
     0; 0; 0;    % ángulos de Euler iniciales (roll, pitch, yaw) [radianes]
     0; 0; 0];   % velocidades angulares iniciales sobre el dron (p, q, r) [rad/s]

% ======================= Vectores historicos ================

X_hist = zeros(12, N); % Vector de historial de variables y comportamiento del dron
X_hist(:,1) = X;  %En su primera columna es igual al vector de estados iniciales
U_hist=zeros(4,N); %Matriz de historial de variables de control

% Historiales para analiis
refs_hist      = zeros(4,N);     % [roll_ref pitch_ref r_ref throttle]
pqr_ref_hist   = zeros(3,N);     % [p_ref q_ref r_ref]
alpha_hist     = zeros(3,N);     % [alpha_p alpha_q alpha_r]
tau_unsat_hist = zeros(3,N);     % τ antes de saturar
tau_sat_hist   = zeros(3,N);     % τ después de saturar (= U(2:4))
U1_hist        = zeros(1,N);     % U1 (redundante con U_hist(1,:), pero cómodo)
R33_hist       = zeros(1,N);     % cos(phi)*cos(theta) (si querés monitorearlo)
w2_eff_hist = zeros(4,N);   % Historial de velocidades angulares teniendo en cuenta 1000-2000uS  
f_eff_hist  = zeros(4,N);   % Historial de fuerza de empuje con clap 1000-2000uS
pwm_raw_hist= zeros(4,N);   %PWM en crudo
nr_it_hist  = zeros(4,N);   % iteraciones de Newton por motor
nr_ok_hist  = false(4,N);   
w2_hist = zeros(4,N);       
w_hist = zeros(4,N); 
f_hist = zeros(4,N); 
pwm_hist = zeros(4,N);

I_mot_hist   = zeros(4,N);
I_total_hist = zeros(1,N);
P_pack_hist  = zeros(1,N);


% ===== Mixer (una vez) =====
M = mixer_matrix_w2_X(Kt, Kq, l);   % 4x4
A_mixer = M.'*M;                    % Simetrica para CG

% ============================ RK4 ===============================

for k = 2:N
   
        ti = t(k-1);

% ==== Escenario 1: step de throttle para probar hover ====
if     ti < 2.0
    % Antes del paso: todo en cero (en el piso)
    refs.roll_ref  = 0.0;
    refs.pitch_ref = 0.0;
    refs.r_ref     = 0.0;
    refs.throttle  = 0.0;
    
elseif ti >= 2.0 && ti < 6.0
    % Paso de throttle (hover / trepada suave)
    refs.roll_ref  = 0.0;
    refs.pitch_ref = 0.0;
    refs.r_ref     = 0.0;
    refs.throttle  = 0.4;   % ajustá este valor hasta que "casi" hovee
    
% ==== Escenario 2: paso de roll para probar actitud ====
elseif  ti >= 8.0 && ti < 10.0
    refs.roll_ref  = deg2rad(5);   % paso de +5 grados en roll
    refs.pitch_ref = 0.0;
    refs.r_ref     = 0.0;
    refs.throttle  = 0.4;

elseif ti >= 10.0 && ti < 12.0
    refs.roll_ref  = deg2rad(-5);  % paso a -5 grados (respuesta simétrica)
    refs.pitch_ref = 0.0;
    refs.r_ref     = 0.0;
    refs.throttle  = 0.4;

% ==== Escenario 3: paso de yaw-rate para probar control en guiñada ====
elseif ti >= 14.0 && ti < 18.0
    refs.roll_ref  = 0.0;
    refs.pitch_ref = 0.0;
    refs.r_ref     = 0.5;          % [rad/s] ≈ 28.6 °/s  (ajustá según tu PID)
    refs.throttle  = 0.0;          % seguimos en hover mientras giramos
else
    % Vuelta a hover plano
    refs.roll_ref  = 0.0;
    refs.pitch_ref = 0.0;
    refs.r_ref     = 0.0;
    refs.throttle  = 0.0;
end



% ==== Control en cascada → U = [U1; τx; τy; τz] ====
[U, parametros_ctrl, mon] = control_cascada(refs, X, phys, parametros_ctrl, h);

% -- Mixer por CG: U -> w^2 pedido --
U_vec = U(:);                       % [U1; tau_x; tau_y; tau_z]
c_m   = M.' * U_vec;
[w2_i, ~] = Gradiente_conjugado(A_mixer, c_m, zeros(4,1), 4, false);
w2_i = max(w2_i, 0);
w_i  = sqrt(w2_i);

% -- PWM "pedido" por motor: uS = a + b*w + c*w^2 --
pwm_i_raw = zeros(4,1);
for i = 1:4
    pwm_i_raw(i) = pwm_from_w_poly(w_i(i), pwm_poly(i,:)); % sin clamp
end
pwm_i = min(max(pwm_i_raw, us_min), us_max);               % clamp físico

% -- Invertir PWM clamp → w efectiva con Newton-Raphson por motor --
w_eff = zeros(4,1);
for i = 1:4
    w_guess = w_i(i);  % buena semilla
    [w_eff(i), itNR, okNR] = w_from_pwm_poly_NR(pwm_i(i), pwm_poly(i,:), w_guess);
    nr_it_hist(i,k) = itNR; 
    nr_ok_hist(i,k) = okNR;
end

% ===== Corriente efectiva por motor a partir de w_eff =====
I_eff = zeros(4,1);
for i = 1:4
    I_eff(i) = I_from_w_poly(w_eff(i), corriente_cuadratica(i,:));
end

% ===== Limite de corriente por motor (10 A) =====
if enforce_Imax
    for i = 1:4
        if I_eff(i) > I_max_motor
            % Encontrar w_lim tal que I_from_w_poly(w_lim) = I_max_motor
            w_lim = w_from_I_poly_quadratic(I_max_motor, corriente_cuadratica(i,:));
            % Aplicar límite
            w_eff(i) = min(w_eff(i), w_lim);
            % Recalcular corriente ya limitada
            I_eff(i) = I_from_w_poly(w_eff(i), corriente_cuadratica(i,:));
            % Opcional: actualizar PWM mostrado (lo re-calculamos desde w limitada)
            pwm_i(i) = min(max(pwm_from_w_poly(w_eff(i), pwm_poly(i,:)), us_min), us_max);
        end
    end
end

% ===== efectivos =====
w2_eff = w_eff.^2;
f_eff  = Kt * w2_eff;

% Logs de corriente y potencia
I_mot_hist(:,k) = I_eff;
I_total_hist(k) = sum(I_eff);
P_pack_hist(k)  = V_pack * I_total_hist(k);

% -- Logs pedidos vs efectivos --
w2_hist(:,k)     = w2_i;
w_hist(:,k)      = w_i;
f_hist(:,k)      = Kt*w2_i;

pwm_raw_hist(:,k)= pwm_i_raw;
pwm_hist(:,k)    = pwm_i;
w2_eff_hist(:,k) = w2_eff;
f_eff_hist(:,k)  = f_eff;

% -- Logs de control --
U_hist(:,k)      = U;
refs_hist(:,k)   = [refs.roll_ref; refs.pitch_ref; refs.r_ref; refs.throttle];
pqr_ref_hist(:,k)= [mon.p_ref; mon.q_ref; refs.r_ref];
alpha_hist(:,k)  = mon.alpha_des(:);
tau_unsat_hist(:,k)= mon.tau_unsat(:);
tau_sat_hist(:,k)= U(2:4);
U1_hist(k)       = U(1);
R33_hist(k)      = mon.R33;

% -- Integración RK4 (elegir UNA)
% X = rk4_paso(X, U,     phys, h);   % ideal (sin límites de actuador)
X = rk4_paso(X, U_eff_from_forces(f_eff, Kt, Kq, l), phys, h); % realista
X_hist(:,k) = X;

    if k == 2
        dx_test = dron_derivadas_fisicas(X_hist(:,1), U, phys);
        if ~isequal(size(dx_test), [12,1])
            error('dx no es 12x1');
        end
    end
end

% ===== Integración Simpson 1/3 (acumulada) =====


% Integral(I_total dt) -> [A*s]; convertir a mAh con /3600*1000
As_cum  = cumulative_simpson13(I_total_hist, h);       % [A*s]
mAh_cum = As_cum * (1000/3600);                        % [mAh]
mAh_cum_hist = mAh_cum;

% Energía: Integral(P dt) -> [W*s] => Wh con /3600
Ws_cum  = cumulative_simpson13(P_pack_hist, h);        % [W*s]
Wh_cum  = Ws_cum / 3600;                               % [Wh]
Wh_cum_hist = Wh_cum;


% Resumen por consola
fprintf('\n=== RESUMEN DE CONSUMO ===\n');
fprintf('Consumo total: %.1f mAh\n', mAh_cum_hist(end));
fprintf('Energía total: %.2f Wh (a V_pack = %.2f V)\n', Wh_cum_hist(end), V_pack);
fprintf('Corriente pico total: %.2f A\n', max(I_total_hist));
fprintf('Corriente pico por motor: [%s] A\n', num2str(max(I_mot_hist,[],2).','%.2f '));

%%
%% ============================ Gráficos ==========================

% ---------- Paleta de colores útil ----------
col_base = lines(6);
col_real = [0 0.4470 0.7410];  % azul MATLAB
col_hat  = [0.8500 0.3250 0.0980]; % naranja

%% 1) Estados: posición y velocidad
figure('Name','Estados: Posición y Velocidad','Color', bg);
tiledlayout(2,3,'Padding','compact','TileSpacing','compact');
nexttile; plot(t, X_hist(1,:),'LineWidth',1.6); ylabel('x [m]'); xlabel('t [s]');
nexttile; plot(t, X_hist(2,:),'LineWidth',1.6); ylabel('y [m]'); xlabel('t [s]');
nexttile; plot(t, X_hist(3,:),'LineWidth',1.6); ylabel('z [m]'); xlabel('t [s]');
nexttile; plot(t, X_hist(4,:),'LineWidth',1.6); ylabel('vx [m/s]'); xlabel('t [s]');
nexttile; plot(t, X_hist(5,:),'LineWidth',1.6); ylabel('vy [m/s]'); xlabel('t [s]');
nexttile; plot(t, X_hist(6,:),'LineWidth',1.6); ylabel('vz [m/s]'); xlabel('t [s]');
beautify_current_figure();

%% 2) Actitud (Euler) vs referencias
figure('Name','Actitud (Euler) vs Referencias','Color', bg);
tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
eul_deg = rad2deg(X_hist(7:9,:)');
roll_ref_deg  = rad2deg(refs_hist(1,:));
pitch_ref_deg = rad2deg(refs_hist(2,:));
nexttile;
    plot(t,eul_deg(:,1),'LineWidth',1.6,'Color',col_base(1,:)); hold on;
    plot(t,roll_ref_deg,'--','LineWidth',1.6,'Color',col_base(5,:));
    ylabel('\phi [deg]'); xlabel('t [s]'); legend('\phi','\phi_{ref}','Location','best');
nexttile;
    plot(t,eul_deg(:,2),'LineWidth',1.6,'Color',col_base(2,:)); hold on;
    plot(t,pitch_ref_deg,'--','LineWidth',1.6,'Color',col_base(6,:));
    ylabel('\theta [deg]'); xlabel('t [s]'); legend('\theta','\theta_{ref}','Location','best');
nexttile;
    plot(t,eul_deg(:,3),'LineWidth',1.6,'Color',col_base(3,:));
    ylabel('\psi [deg]'); xlabel('t [s]');
beautify_current_figure();

%% 3) Tasas (p,q,r) vs referencias
figure('Name','Tasas (p,q,r) vs Referencias','Color', bg);
tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
nexttile;
    plot(t, X_hist(10,:),'LineWidth',1.6,'Color',col_base(1,:)); hold on;
    plot(t, pqr_ref_hist(1,:),'--','LineWidth',1.6,'Color',col_base(5,:));
    ylabel('p [rad/s]'); xlabel('t [s]'); legend('p','p_{ref}','Location','best');
nexttile;
    plot(t, X_hist(11,:),'LineWidth',1.6,'Color',col_base(2,:)); hold on;
    plot(t, pqr_ref_hist(2,:),'--','LineWidth',1.6,'Color',col_base(6,:));
    ylabel('q [rad/s]'); xlabel('t [s]'); legend('q','q_{ref}','Location','best');
nexttile;
    plot(t, X_hist(12,:),'LineWidth',1.6,'Color',col_base(3,:)); hold on;
    plot(t, pqr_ref_hist(3,:),'--','LineWidth',1.6,'Color',col_base(4,:));
    ylabel('r [rad/s]'); xlabel('t [s]'); legend('r','r_{ref}','Location','best');
beautify_current_figure();

%% 4) Aceleraciones angulares deseadas
figure('Name','Aceleraciones angulares deseadas','Color', bg);
plot(t, alpha_hist','LineWidth',1.6);
xlabel('t [s]'); ylabel('\alpha [rad/s^2]');
legend('\alpha_p','\alpha_q','\alpha_r','Location','best');
beautify_current_figure();

%% 5) Torques (antes y después de saturación)
figure('Name','Torques','Color', bg);
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t, tau_unsat_hist','--','LineWidth',1.6); ylabel('\tau_{unsat} [N·m]');
           legend('\tau_x','\tau_y','\tau_z','Location','best');
nexttile; plot(t, tau_sat_hist','-','LineWidth',1.8);  ylabel('\tau_{sat} [N·m]'); xlabel('t [s]');
           legend('\tau_x','\tau_y','\tau_z','Location','best');
beautify_current_figure();

%% 6) Empuje y throttle
figure('Name','Empuje y Throttle','Color', bg);
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t, U1_hist, 'LineWidth',1.8); ylabel('U_1 [N]');
nexttile; plot(t, refs_hist(4,:), 'LineWidth',1.6); ylabel('throttle [-]'); xlabel('t [s]');
beautify_current_figure();

%% 7) R33 (proyección vertical del empuje)
figure('Name','R33','Color', bg);
plot(t, R33_hist, 'LineWidth',1.6);
xlabel('t [s]'); ylabel('R_{33} = cos\phi cos\theta');
beautify_current_figure();

%% 8) Mixer: fuerzas, velocidades, PWM
% Fuerzas por motor
figure('Name','Fuerzas por motor','Color', bg);
plot(t, f_hist','LineWidth',1.6);
xlabel('t [s]'); ylabel('f_i [N]');
legend('f_1','f_2','f_3','f_4','Location','best');
beautify_current_figure();

% Velocidades por motor
figure('Name','Velocidades por motor','Color', bg);
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t, w_hist','LineWidth',1.6);
         ylabel('\omega_i [rad/s]'); legend('\omega_1','\omega_2','\omega_3','\omega_4','Location','best');
nexttile; RPM = w_hist' * (60/(2*pi));
         plot(t, RPM,'LineWidth',1.6);
         ylabel('RPM'); xlabel('t [s]'); legend('m1','m2','m3','m4','Location','best');
beautify_current_figure();

% Velocidades al cuadrado
figure('Name','\omega^2 por motor','Color', bg);
plot(t, w2_hist','LineWidth',1.6);
xlabel('t [s]'); ylabel('\omega_i^2 [(rad/s)^2]');
legend('\omega_1^2','\omega_2^2','\omega_3^2','\omega_4^2','Location','best');
beautify_current_figure();

% PWM por motor
figure('Name','PWM por motor uS','Color', bg);
plot(t, pwm_hist','LineWidth',1.6);
xlabel('t [s]'); ylabel('PWM [\mus]');
legend('m1','m2','m3','m4','Location','best');
beautify_current_figure();

%% 9) Consistency check: U ≈ M*w^2
ls = l/sqrt(2);
U1_hat   = Kt*( w2_hist(1,:) + w2_hist(2,:) + w2_hist(3,:) + w2_hist(4,:) );
tauX_hat = Kt*ls*( -w2_hist(1,:) + w2_hist(2,:) - w2_hist(3,:) + w2_hist(4,:) );
tauY_hat = Kt*ls*( -w2_hist(1,:) - w2_hist(2,:) + w2_hist(3,:) + w2_hist(4,:) );
tauZ_hat = Kq*(  w2_hist(1,:) - w2_hist(2,:) + w2_hist(3,:) - w2_hist(4,:) );

figure('Name','Chequeo U vs M*w^2','Color', bg);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;
    plot(t, U_hist(1,:), 'LineWidth',1.9, 'Color', col_real); hold on;
    plot(t, U1_hat,      '--', 'LineWidth',1.9, 'Color', col_hat);
    ylabel('U_1 [N]'); legend('U_1','\hat U_1','Location','best');
nexttile;
    plot(t, U_hist(2,:), 'LineWidth',1.9, 'Color', col_real); hold on;
    plot(t, tauX_hat,    '--', 'LineWidth',1.9, 'Color', col_hat);
    ylabel('\tau_x [N·m]'); legend('\tau_x','\hat \tau_x','Location','best');
nexttile;
    plot(t, U_hist(3,:), 'LineWidth',1.9, 'Color', col_real); hold on;
    plot(t, tauY_hat,    '--', 'LineWidth',1.9, 'Color', col_hat);
    ylabel('\tau_y [N·m]'); legend('\tau_y','\hat \tau_y','Location','best');
nexttile;
    plot(t, U_hist(4,:), 'LineWidth',1.9, 'Color', col_real); hold on;
    plot(t, tauZ_hat,    '--', 'LineWidth',1.9, 'Color', col_hat);
    ylabel('\tau_z [N·m]'); legend('\tau_z','\hat \tau_z','Location','best'); xlabel('t [s]');
beautify_current_figure();

% Error de reconstrucción
errU = [U1_hat - U_hist(1,:);
        tauX_hat - U_hist(2,:);
        tauY_hat - U_hist(3,:);
        tauZ_hat - U_hist(4,:)];
err_norm = sqrt(sum(errU.^2,1));
figure('Name','Error ||U - M w^2||','Color', bg);
plot(t, err_norm,'LineWidth',1.8);
xlabel('t [s]'); ylabel('||error||_2');
beautify_current_figure();

%% 10) PWM pedido vs clamped
figure('Name','PWM pedido vs clamped','Color', bg);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
for i=1:4
    nexttile;
    plot(t, pwm_raw_hist(i,:), 'LineWidth',1.6); hold on;
    plot(t, pwm_hist(i,:),     '--', 'LineWidth',1.9);
    yline(us_min,':'); yline(us_max,':');
    title(sprintf('Motor %d',i));
    ylabel('PWM [\mus]'); if i>=3, xlabel('t [s]'); end
    legend('pedido','clamped','Location','best');
end
beautify_current_figure();

%% 11) ω pedido vs ω efectivo (NR)
figure('Name','\omega pedido vs \omega efectivo','Color', bg);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
for i=1:4
    nexttile;
    plot(t, w_hist(i,:), 'LineWidth',1.6); hold on;
    plot(t, sqrt(w2_eff_hist(i,:)), '--', 'LineWidth',1.9);
    title(sprintf('Motor %d',i));
    ylabel('\omega [rad/s]'); if i>=3, xlabel('t [s]'); end
    legend('\omega_{ped}','\omega_{eff}','Location','best');
end
beautify_current_figure();

%% 12) Fuerza pedida vs efectiva
figure('Name','Fuerza pedida vs efectiva','Color', bg);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
for i=1:4
    nexttile;
    plot(t, f_hist(i,:), 'LineWidth',1.6); hold on;
    plot(t, f_eff_hist(i,:), '--', 'LineWidth',1.9);
    title(sprintf('Motor %d',i));
    ylabel('f [N]'); if i>=3, xlabel('t [s]'); end
    legend('pedido','efectivo','Location','best');
end
beautify_current_figure();

%% 13) NR: g(w) = a + b w + c w^2 - PWM (semilla y solución)
k0 = round(N/2);
kk = find(any(pwm_hist>us_min & pwm_hist<us_max,1), 1, 'last');
if ~isempty(kk), k0 = kk; end

figure('Name','NR: g(w) por motor (semilla y solución)','Color', bg);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
for i=1:4
    a = pwm_poly(i,1); b = pwm_poly(i,2); c = pwm_poly(i,3);
    pwm_k  = pwm_hist(i,k0);
    w_seed = w_hist(i,k0);
    w_sol  = sqrt(w2_eff_hist(i,k0));
    g  = @(w) (a + b*w + c*w.^2) - pwm_k;

    w_max = max([1.5*w_sol, 1.5*w_seed, 4000]);
    w_vec = linspace(0, w_max, 400);
    g_vec = g(w_vec);

    nexttile; hold on;
    plot(w_vec, g_vec, 'LineWidth',1.9);
    yline(0, ':', 'LineWidth',1.2);
    xline(w_seed, '--', 'Color',[0.85 0.33 0.10], 'LineWidth',1.4);
    xline(w_sol,  '-',  'Color',[0.00 0.45 0.74], 'LineWidth',1.9);
    scatter([w_seed w_sol], [g(w_seed) 0], 28, 'filled');
    title(sprintf('Motor %d — t=%.2fs, PWM=%g μs', i, t(k0), pwm_k));
    xlabel('\omega [rad/s]'); ylabel('g(\omega)');
    legend('g(\omega)','y=0','\omega_{seed}','\omega_{sol}','Location','best');
end
beautify_current_figure();

%% 14) Corriente por motor
figure('Name','Corriente por motor','Color', bg);
plot(t, I_mot_hist','LineWidth',1.8);
xlabel('t [s]'); ylabel('I_i [A]');
legend('m1','m2','m3','m4','Location','best');
yline(I_max_motor,':r','I_{max}');
beautify_current_figure();

%% 15) Corriente total y potencia
figure('Name','Corriente total y potencia','Color', bg);
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t, I_total_hist,'LineWidth',1.9); ylabel('I_{tot} [A]');
nexttile; plot(t, P_pack_hist,'LineWidth',1.9); ylabel('P [W]'); xlabel('t [s]');
beautify_current_figure();

%% 16) Consumo acumulado
figure('Name','Consumo acumulado','Color', bg);
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t, mAh_cum_hist,'LineWidth',2.0); ylabel('mAh acumulados');
nexttile; plot(t, Wh_cum_hist,'LineWidth',2.0); ylabel('Wh acumulados'); xlabel('t [s]');
beautify_current_figure();

%% 17) Consumo con Simpson 1/3: áreas sombreadas (sin mosquitera)
% Totales desde tus históricos
I_tot = I_total_hist;           % 1 x N
P_tot = P_pack_hist;            % 1 x N

% Corriente total — área integrada
figure('Name','Corriente total — Área integrada (Simpson 1/3)','Color', bg);
hold on;
shade(t, I_tot, [0 0.45 0.74]);
plot(t, I_tot, 'k', 'LineWidth',1.9);
xlabel('t [s]'); ylabel('I_{tot} [A]');
title('Área bajo I_{tot}(t)');
% marcas Simpson: máximo 8 líneas, separadas cada varios pasos
nLines = min(8, floor((numel(t)-1)/2));
if nLines >= 2
    step2 = floor((numel(t)-1)/(2*nLines))*2;        % múltiplo de 2
    idxS  = 1:step2:numel(t);
    xline(t(idxS), ':', 'Color',[0.45 0.45 0.45], 'LineWidth',0.8, 'HandleVisibility','off');
end
beautify_current_figure();

% Potencia instantánea — área integrada
figure('Name','Potencia instantánea — Área integrada (energía)','Color', bg);
hold on;
shade(t, P_tot, [0.85 0.33 0.10]);
plot(t, P_tot, 'k', 'LineWidth',1.9);
xlabel('t [s]'); ylabel('P [W]');
title('Área bajo P(t) = V_{bat} I_{tot}(t)');
if exist('idxS','var')
    xline(t(idxS), ':', 'Color',[0.45 0.45 0.45], 'LineWidth',0.8, 'HandleVisibility','off');
end
beautify_current_figure();

%% =================== Vista 3D simple: trayectoria + dron ===================

% --- 1) Preparar muestras para animación (suave y acotada) ---
factor_frames = 8;                                   % más alto = más suave
t_anim  = linspace(t(1), t(end), (numel(t)-1)*factor_frames + 1);
X_anim  = interpolar_spline_natural_multicanal(t, X_hist, t_anim);  % 12 x K

% Limitar cantidad de cuadros para sims MUY largas (fluida ~20–25 s)
fps      = 60;
K_target = 1400;
decim    = max(1, ceil((numel(t_anim)-1)/K_target));
idx      = 1:decim:numel(t_anim);
t_anim   = t_anim(idx);
X_anim   = X_anim(:,idx);
K        = numel(t_anim);
dt       = 1/fps;

% --- 2) Figura y ejes (tema claro, cámara FIJA) ---
fig3d = figure('Color','w','Units','pixels','Position',[120 90 1100 720]);
ax    = axes('Parent',fig3d); hold(ax,'on');
set(ax,'Color','w','XColor',[0 0 0],'YColor',[0 0 0],'ZColor',[0 0 0], ...
       'GridColor',[0.75 0.75 0.75],'MinorGridColor',[0.90 0.90 0.90], ...
       'GridAlpha',0.6,'MinorGridAlpha',0.6,'LineWidth',0.9,'Box','off');
grid(ax,'on'); grid(ax,'minor');
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
view(ax,35,25); daspect(ax,[1 1 1]); axis(ax,'vis3d');

% --- 3) Caja CÚBICA para ver toda la trayectoria (misma escala en X,Y,Z) ---
P    = X_hist(1:3,:);                    % posiciones de toda la simulación
mins = min(P,[],2);
maxs = max(P,[],2);
span = maxs - mins;                      % ancho por eje
c    = (mins + maxs)/2;                  % centro de la caja

Lmin = 0.5;                              % lado mínimo de la caja (m)
L     = max([span; Lmin]);               % lado = mayor span o Lmin
hL    = L/2;                             % medio lado

xlim(ax, [c(1)-hL, c(1)+hL]);
ylim(ax, [c(2)-hL, c(2)+hL]);
zlo = max(0, c(3)-hL);                   % opcional: no bajar de z=0
zlim(ax, [zlo, zlo + L]);

% --- piso (opcional) ---
[Xg,Yg] = meshgrid(linspace(c(1)-hL, c(1)+hL, 2), linspace(c(2)-hL, c(2)+hL, 2));
Zg = zlo*ones(size(Xg));
surf(ax,Xg,Yg,Zg,'FaceColor',[0.95 0.95 0.95],'EdgeColor','none','FaceAlpha',0.6);


view(ax,35,25);
daspect(ax,[1 1 1]);                     % relación 1:1:1
axis(ax,'vis3d');                        % bloquea distorsión al rotar
title(ax,'Preview 3D — actitud y trayectoria','Color',[0 0 0]);


% --- 4) Trazo COMPLETO de la trayectoria (una sola vez) ---
plot3(ax, X_hist(1,:), X_hist(2,:), X_hist(3,:), 'k-', 'LineWidth', 1.4);

% (opcional) marcador de posición que se mueve
hPt = plot3(ax, X_hist(1,1), X_hist(2,1), X_hist(3,1), 'ko', ...
            'MarkerFaceColor','k','MarkerSize',5);

% --- 5) Cargar STL y colocar dron (oscuro/metal) ---
stl_path   = 'DRON EXPO v1.stl';
Escala_a_m = 0.001;
[hT, hMesh] = load_dron(ax, stl_path, Escala_a_m, [0.12 0.12 0.14]);  % base oscura
set(hMesh,'FaceColor',[0.12 0.12 0.14],'EdgeColor','none', ...
          'FaceLighting','gouraud','BackFaceLighting','reverselit', ...
          'AmbientStrength',0.25,'DiffuseStrength',0.70, ...
          'SpecularStrength',0.30,'SpecularExponent',35);
lighting(ax,'gouraud'); camlight(ax,'headlight'); camlight(ax,'right');
R_mesh_to_body = eye(3);

% --- 6) Animación: cámara FIJA, el dron "sigue" la trayectoria ---
for k = 1:K
    Xk = X_anim(:,k);
    rk = Xk(1:3);                                    % posición
    Rk = matriz_rotacion_cuerpo_al_mundo(Xk(7), Xk(8), Xk(9));  % actitud

    % actualizar dron y punto
    Correcion_pose(hT, Rk, rk, R_mesh_to_body);
    set(hPt,'XData',rk(1),'YData',rk(2),'ZData',rk(3));

    drawnow limitrate; pause(dt);
end
end

 

% ================= Helpers locales =================

function beautify_current_figure()
    axs = findall(gcf,'Type','axes'); if isempty(axs), return; end
    for i = 1:numel(axs), nice_axes(axs(i)); end
end

function nice_axes(ax)
    if nargin==0, ax = gca; end
    ax.Color       = [0.08 0.08 0.10];
    ax.XColor      = [1 1 1];
    ax.YColor      = [1 1 1];
    ax.LineWidth   = 0.9;
    grid(ax,'on');
    ax.GridColor   = [0.40 0.40 0.40];
    ax.GridAlpha   = 0.25;
    ax.XMinorGrid  = 'off';
    ax.YMinorGrid  = 'off';
    box(ax,'off');

    % Títulos/labels en blanco (que no queden gris claro)
    ax.Title.Color   = [1 1 1];
    ax.XLabel.Color  = [1 1 1];
    ax.YLabel.Color  = [1 1 1];
end

function shade(x,y,c)
    % Área translúcida, sin borde
    area(x, y, 'FaceColor', c, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
end



function u = pwm_from_w_poly(w, poly)
a = poly(1); b = poly(2); c = poly(3);
u = a + b*w + c*(w.^2);
end

function [x, it, ok] = newton_scalar(g, dg, x0, tol, kmax)
if nargin<4, tol=1e-9; end
if nargin<5, kmax=50; end
x = x0; ok = false;
for it=1:kmax
    gx = g(x); dgx = dg(x);
    if abs(dgx) < 1e-12, break; end
    x_new = x - gx/dgx;
    if abs(x_new - x) <= tol*(1+abs(x)), x = max(x_new,0); ok = true; return; end
    x = x_new;
end
x = max(x,0);
end

function [w, it, ok] = w_from_pwm_poly_NR(pwm, poly, w_guess)
a = poly(1); b = poly(2); c = poly(3);
g  = @(w) (a + b*w + c*w.^2) - pwm;
dg = @(w) b + 2*c*w;
[w, it, ok] = newton_scalar(g, dg, w_guess, 1e-9, 50);
if ~ok
    if abs(c) < 1e-12
        w = max((pwm - a)/max(b,1e-12), 0);
    else
        D = b^2 - 4*c*(a - pwm);
        D = max(D, 0);
        w = max((-b + sqrt(D))/(2*c), 0);
    end
    it = NaN;
end
end

function U_eff = U_eff_from_forces(f, Kt, Kq, l)
ls = l/sqrt(2);
U_eff = [ sum(f);
          ls*(-f(1)+f(2)-f(3)+f(4));
          ls*(-f(1)-f(2)+f(3)+f(4));
          (Kq/Kt)*( f(1)-f(2)+f(3)-f(4) ) ];
end

function I = I_from_w_poly(w, polyI)
% I = aI + bI*w + cI*w^2
aI = polyI(1); bI = polyI(2); cI = polyI(3);
I  = aI + bI*w + cI*(w.^2);
I  = max(I,0);   % por seguridad
end

function w_lim = w_from_I_poly_quadratic(I_target, polyI)
% Resuelve aI + bI*w + cI*w^2 = I_target (elige la raíz >= 0)
aI = polyI(1); bI = polyI(2); cI = polyI(3);
A = cI; B = bI; C = aI - I_target;
if abs(A) < 1e-14
    % Caso lineal: B*w + C = 0
    if abs(B) < 1e-14, w_lim = 0; else, w_lim = max(-C/B, 0); end
else
    D = max(B*B - 4*A*C, 0);
    w1 = (-B + sqrt(D))/(2*A);
    w2 = (-B - sqrt(D))/(2*A);
    w_lim = max([w1, w2, 0]);   % tomamos la raíz no negativa más grande
end
end

function F = cumulative_simpson13(y, h)
% Integral acumulada con compuesto de Simpson 1/3 sobre malla uniforme.
% F(k) ≈ ∫_0^{t_k} y(t) dt
n = numel(y);
F = zeros(1,n);
if n < 2, return; end
for k = 2:n
    if mod(k-1,2) == 0
        % k-1 = número de intervalos par -> aplicar Simpson en los últimos 2
        F(k) = F(k-2) + (h/3)*( y(k-2) + 4*y(k-1) + y(k) );
    else
        % número de intervalos impar -> usar trapecio en el último
        F(k) = F(k-1) + (h/2)*( y(k) - y(k-1) );
    end
end
end




