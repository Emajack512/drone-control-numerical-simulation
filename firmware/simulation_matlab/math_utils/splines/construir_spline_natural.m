function spl = construir_spline_natural(X, Y)
% Spline cúbico natural 1D
% Entradas:
%   X : 1xN o Nx1 (nodos, crecientes y distintos)
%   Y : 1xN o Nx1 (valores)
% Salida:
%   spl : struct con campos X, Y, B, C, D (A≡Y), H, n

X = X(:).';  % fila
Y = Y(:).';  % fila
N = numel(X);
if N < 2, error('Se requieren al menos 2 puntos.'); end
if any(diff(X) <= 0), error('X debe ser estrictamente creciente.'); end

n = N - 1;                  % cantidad de tramos
H = diff(X);                % H(i) = X(i+1)-X(i)

% Matriz tridiagonal A (natural: c0=0, cn=0) y vector b
A = zeros(N, N);
b = zeros(N, 1);

A(1,1) = 1;                 % c0 = 0
A(N,N) = 1;                 % cn = 0
for i = 2:n
    A(i, i-1) = H(i-1);
    A(i, i)   = 2*(H(i-1) + H(i));
    A(i, i+1) = H(i);
    b(i) = 3*((Y(i+1) - Y(i))/H(i) - (Y(i) - Y(i-1))/H(i-1));
end

% Coeficientes C resolviendo A*C = b  (mejor que inv(A)*b)
C = A \ b;

% Coeficientes B y D por tramo (A ≡ Y)
B = zeros(1,n);
D = zeros(1,n);
for i = 1:n
    B(i) = (Y(i+1)-Y(i))/H(i) - (2*C(i)+C(i+1))*H(i)/3;
    D(i) = (C(i+1)-C(i)) / (3*H(i));
end

spl.X = X; spl.Y = Y; spl.B = B; spl.C = C; spl.D = D; spl.H = H; spl.n = n;
end
