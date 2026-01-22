function T = pose_to_T(R, r0)
% R: 3x3 body->world
% r0: 3x1 posicion en mundo
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4)   = r0(:);
end
