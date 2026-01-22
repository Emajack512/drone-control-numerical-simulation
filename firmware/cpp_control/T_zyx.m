function T = T_zyx(roll, pitch)
    cph = cos(roll); sph = sin(roll);
    cth = cos(pitch); sth = sin(pitch);
    eps = 1e-8; if cth < eps, cth = eps; end
    T = [ 1,  sph*sth/cth,  cph*sth/cth;
          0,  cph,         -sph;
          0,  sph/cth,      cph/cth ];
end
