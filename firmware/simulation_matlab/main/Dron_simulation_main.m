%Codigo de simulacion del dron 

clc; clear all; close all;

%Parametros del dron

% Tensor de inercia 3x3
I=[2.0, 0.0, 0.0;     % I_xx, I_xy, I_xz
   0.0, 2.0, 0.0;     % I_yx, I_yy, I_yz
   0.0, 0.0, 2.0];    % I_zx, I_zy, I_zz

l = 0.2; %distancia de brazo del centro del motor al centro de masa
m=1.5;  % Masa del dron en Kg
Kt=0.3; %relacion entre el empuje total y la velocidad angular de las helices
S=[1,-1,1,-1]; %secuencia de par de arrastre del sentido de giro 
Kq=0.1; %relacion lineal entre el Tz y la velocidad angular de las helices
Kw=1; %relacion lineal entre las señal de control PWM en uS y la velocidad de las helices

% Parametros del medio
g=9.81; %gravedad en m/s^2

% Parametros de simulacion RK4

h=0.01; %paso de simulacion RK4
T_total = input('Ingrese la cantidad de tiempo de simulacion: '); %Tiempo total de simulacion
N= round(T_total/h);  %Cantidad de pasos de la simulacion

% Estado inicial
X = [0; 0; 0;    % posición inicial (x, y, z) [metros]
     0; 0; 0;    % velocidad inicial (vx, vy, vz) [m/s]
     0; 0; 0;    % ángulos de Euler iniciales (roll, pitch, yaw) [radianes]
     0; 0; 0];   % velocidades angulares iniciales (p, q, r) [rad/s]

%Vector de control
U=[0;0;0;0]; %Empuje total, Tx,Ty,Tz

dx = quad_f(X, U, I, m, g)   % <<<< que esta línea te devuelva un vector 12x1 coherente

X_hist=zeros(12,N);
X_hist(:,1)=X; %primera posicion inicial, estados iniciales

for k=2:N

    X= Paso_RK4(X, U, h, I, m, g);
    X_hist(:,k)=X;
end 

function dx = quad_f(X, U, I,m ,g)
    % extrae estados
    px=X(1); py=X(2); pz=X(3);
    vx=X(4); vy=X(5); vz=X(6);
    roll=X(7); pitch=X(8); yaw=X(9);
    p=X(10); q=X(11); r=X(12);

    % entradas
    U1 = U(1); tau = U(2:4);  %empuje y Tao

    % Constantes cosenos
    croll = cos(roll); sroll = sin(roll);
    cpitch  = cos(pitch);  spitch  = sin(pitch);
    cyaw = cos(yaw); syaw = sin(yaw);
    
    eps_sing = 1e-6;
    if abs(cpitch) < eps_sing
        % evitar dividir por ~0 (tan(theta) y sec(theta))
        % opción simple para ahora: saturar
        cpitch = sign(cpitch)*eps_sing;
    end

    % matriz R que nos permite pasar del sistema movil al sistema fijo ZYX
    R = [ cyaw*cpitch,  cyaw*spitch*sroll - syaw*croll,  cyaw*spitch*croll + syaw*sroll;
          syaw*cpitch,  syaw*spitch*sroll + cyaw*croll,  syaw*spitch*croll - cyaw*sroll;
          -spitch,      cpitch*sroll,                   cpitch*croll ];

    % T matriz que relaciona la velocidad de cambio de roll, pitch y yaw con p,q,r
    % siendo Eta_punto=T*w
    den = max(abs(cpitch), 1e-6);  % solo para divisiones

    T = [ 1, sroll*spitch/den, croll*spitch/den;   
          0, croll,         -sroll;
          0, sroll/den,     croll/den ];

    %W es la matriz que w=W*Eta_punto
    W=inv(T);

    % vectores
    v   = [vx; vy; vz];  %vector de velocidades en el sistema fijo
    w = [p; q; r];  %velocidades angulares en el sistema movil
 
    % derivadas
    dpos = v;
    F    = R*[0;0;U1] + [0;0;-m*g]; %Fuerzas en el sistema fijo
    dv   = F/m; 

    Eta_punto = T*w;

    h = I*w; %calculamos el momento angular del dron 
    dw = inv(I)*(tau - cross(w, h));

    % apila
    dx = [dpos; dv; Eta_punto; dw];
end
function X_next = Paso_RK4(X, U, h, I, m, g)
    k1 = quad_f(X, U, I, m, g);
    k2 = quad_f(X + 0.5*h*k1, U, I, m, g);
    k3 = quad_f(X + 0.5*h*k2, U, I, m, g);
    k4 = quad_f(X + h*k3,    U, I, m, g);
    X_next = X + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end


