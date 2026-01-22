function yq = evaluar_spline_natural(spl, xq)
% Evalúa el spline natural en xq (vector)
X = spl.X; Y = spl.Y; B = spl.B; C = spl.C; D = spl.D; n = spl.n;

xq = xq(:);
yq = zeros(size(xq));

% Opcional: saturar consultas fuera del rango al borde
xq(xq < X(1))   = X(1);
xq(xq > X(end)) = X(end);

for k = 1:numel(xq)
    x = xq(k);

    % hallar tramo i tal que X(i) <= x < X(i+1)
    i = find(X <= x, 1, 'last');
    if i >= n+1, i = n; end   % si cae exacto en X(end), usar último tramo

    dx = x - X(i);
    yq(k) = Y(i) + B(i)*dx + C(i)*dx^2 + D(i)*dx^3;
end

yq = yq(:).';   % devolver fila por comodidad
end
