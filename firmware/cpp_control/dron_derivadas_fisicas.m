function dx = dron_derivadas_fisicas(X, U, phys)

% Extraemos los estados de X
x = X(1);  y = X(2);  z = X(3);
vx= X(4); vy = X(5); vz = X(6);
roll = X(7); pitch = X(8); yaw = X(9);
p = X(10);  q = X(11);   r = X(12);

% Extraemos los parametros de la estructura phys
m = phys.m; g = phys.g; I = phys.I;

% ---- Arrastre (constantes locales, NO en phys) ----
% lineal + cuadrático por componente; actúa solo si |v|>eps
kL = 0.5;          % [N·s/m]   (ajustable)
kQ = 0.5;          % [N·s^2/m^2] (ajustable)
v  = [vx; vy; vz];
eps_v = 1e-6;

drag_comp = @(vi) (abs(vi)>eps_v) .* (kL*vi + kQ*abs(vi).*vi);

% Entradas físicas
U1  = U(1);           % Empuje en el cuerpo [N]
tau = U(2:4);         % [Tx Ty Tz] en el cuerpo [N·m]

% Rotación y cinemática (ZYX)
R = matriz_rotacion_cuerpo_al_mundo(roll, pitch, yaw);    %Obtenemos la matriz de rotacion con los angulos actuales
T = T_zyx(roll, pitch);     % Obtenemos la matriz T que me sirve para saber la relacion entre la derivada de los angulos de euler y las velocidades angulares

% Traslación
F_mundo = R*[0;0;U1];% Podemos saber las fuerzas en el mundo sobre el cuerpo
Fgrav     = [0;0;-m*g]; 

% Arrastre en mundo (aprox. suficiente para traslación moderada)
Fdrag = -[ drag_comp(vx);
           drag_comp(vy);
           drag_comp(vz) ];

acc = (F_mundo + Fgrav + Fdrag) / m; %Podemos saber la aceleracion en el mundo del centro de gravedad/masa del dron

% Rotación: Aplicamos la formula T = I * w_punto + w x ( I * w ), buscamos
% saber el valor de w_punto
w     = [p; q; r];
hvec  = I * w;
w_punto = I \ (tau - cross(w, hvec));

Eta= T * w; %Calculamos el vector Eta de derivadas del los angulos roll, pitch y yaw 

% Empaquetamos derivadas que se usan en RK4
dx = zeros(12,1);
dx(1:3)   = [vx; vy; vz];  % ṙ = v
dx(4:6)   = acc;           % v̇ = a
dx(7:9)   = Eta;         % Eta = T(φ,θ) * [p;q;r]
dx(10:12) = w_punto;       % ẇ
end
