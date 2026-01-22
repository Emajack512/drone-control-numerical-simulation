function [coef, fh, stats] = Minimos_cuadrados(X, Y, grado)
% [coef, fh, stats] = Minimos_cuadrados(X, Y, grado)
% Ajuste por mínimos cuadrados de un polinomio de grado "grado".
% - X, Y: vectores columna o fila (se fuerzan a columna)
% - coef: [a0; a1; ...; a_grado]  (a0 término independiente)
% - fh:   handle de función para evaluar el ajuste: y_hat = fh(x)
% - stats: RMSE y R2 del ajuste

    X = X(:);
    Y = Y(:);

    % Filtrado básico por si hay NaN/Inf
    m = isfinite(X) & isfinite(Y);
    X = X(m); Y = Y(m);

    n = numel(X);
    if numel(Y) ~= n, error('X e Y deben tener la misma longitud.'); end
    if ~(isscalar(grado) && grado>=0 && grado==floor(grado))
        error('grado debe ser un entero >= 0');
    end

    % Matriz de diseño (Vandermonde con a0 primero)
    A = ones(n, grado+1);
    for g = 1:grado
        A(:, g+1) = X.^g;
    end

    % Ecuaciones normales (misma metodología que tu script original, sin inv)
    coef = (A' * A) \ (A' * Y);   % [a0; a1; ...; a_grado]

    % Métricas
    y_hat = A * coef;
    res   = Y - y_hat;
    stats.RMSE = sqrt(mean(res.^2));
    ss_res = sum(res.^2);
    ss_tot = sum((Y - mean(Y)).^2);
    stats.R2 = 1 - ss_res/ss_tot;

    % Handle para evaluar (usa polyval, que pide [a_grado ... a0])
    coef_polyval = flipud(coef).';
    fh = @(x) polyval(coef_polyval, x);

    % Mostrar polinomio de forma robusta
    fprintf('P(x) = ');
    for k = 0:grado
        a = coef(k+1);
        if k == 0
            fprintf('%.6g', a);
        elseif k == 1
            fprintf(' %+ .6g*x', a);
        else
            fprintf(' %+ .6g*x^%d', a, k);
        end
    end
    fprintf('\n');
end
