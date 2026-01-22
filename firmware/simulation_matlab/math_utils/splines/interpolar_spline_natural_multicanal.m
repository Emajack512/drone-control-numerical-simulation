function X_anim = interpolar_spline_natural_multicanal(t_sim, X_sim, t_anim)
% Interpola 12xN (o MxN) con spline natural por canal
% t_sim  : 1xN (creciente)
% X_sim  : MxN (cada fila una señal)
% t_anim : 1xK (tiempos de animación)
% X_anim : MxK

if size(t_sim,1) > 1, t_sim = t_sim(:)'; end
[M, N] = size(X_sim);
K = numel(t_anim);
X_anim = zeros(M, K);

for m = 1:M
    spl = construir_spline_natural(t_sim, X_sim(m,:));
    X_anim(m,:) = evaluar_spline_natural(spl, t_anim);
end
end
