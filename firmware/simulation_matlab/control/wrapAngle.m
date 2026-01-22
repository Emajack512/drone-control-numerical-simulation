function e = wrapAngle(a)
% devuelve a envuelto en [-pi, pi]
e = mod(a + pi, 2*pi) - pi;
end
