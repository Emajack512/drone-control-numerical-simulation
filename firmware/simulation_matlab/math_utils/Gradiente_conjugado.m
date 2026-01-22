function [x, logcg] = Gradiente_conjugado(A, b, x0, kmax, use_trunc6)
% A SPD, b col.  x0 semilla. kmax <= n. use_trunc6=true aplica truncamiento 6 dec.
if nargin<3||isempty(x0), x0 = zeros(size(b)); end
if nargin<4, kmax = size(A,1); end
if nargin<5, use_trunc6 = false; end
tt = @(z) (use_trunc6 .* trunc6(z) + (~use_trunc6).*z);

x  = tt(x0);
r  = tt(b - A*x);
d  = r;

logcg = struct('x',[],'r',[],'d',[],'p',[],'alpha',[]);
for k = 1:kmax
    Ad    = tt(A*d);
    p     = (r.'*r) / (d.'*Ad);
    x_new = tt(x + p*d);
    r_new = tt(r - p*Ad);

    logcg(k).x     = x_new;
    logcg(k).r     = r_new;
    logcg(k).d     = d;
    logcg(k).p     = p;
    logcg(k).alpha = (r_new.'*r_new) / max(r.'*r, eps);

    if norm(r_new) < 1e-12*(1+norm(b)), x = x_new; return; end

    alpha = logcg(k).alpha;
    d     = tt(r_new + alpha*d);
    x     = x_new;  r = r_new;
end
end

function y = trunc6(x)
% trunca (no redondea) a 6 decimales, elemento a elemento
y = fix(x*1e6)./1e6;
end
