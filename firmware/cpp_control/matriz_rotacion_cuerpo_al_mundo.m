function R = matriz_rotacion_cuerpo_al_mundo(roll, pitch, yaw)
cph = cos(roll); sph = sin(roll);
cth = cos(pitch); sth = sin(pitch);
cps = cos(yaw); sps = sin(yaw);
R = [ cps*cth,  cps*sth*sph - sps*cph,  cps*sth*cph + sps*sph;
      sps*cth,  sps*sth*sph + cps*cph,  sps*sth*cph - cps*sph;
      -sth,     cth*sph,                cth*cph ];
end
