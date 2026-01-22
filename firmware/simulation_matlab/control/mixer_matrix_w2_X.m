function M = mixer_matrix_w2_X(Kt, Kq, l)
ls = l/sqrt(2);

M = [ Kt,      Kt,      Kt,      Kt;      % T   (empuje total)
     -Kt*ls,  +Kt*ls,  -Kt*ls,  +Kt*ls;   % tau_x (roll)
     -Kt*ls,  -Kt*ls,  +Kt*ls,  +Kt*ls;   % tau_y (pitch)
      Kq,     -Kq,     -Kq,      Kq ];    % tau_z (yaw) S = [+ - - +]
end
