
% hT: handle hgtransform del mesh
% R_world_body: R (cuerpo->world) de la simulacion
% r_world: r0 (posicion del dron) en mundo
% R_mesh_to_body: rotación fija para alinear el STL con el eje del body

function Correcion_pose(hT, R_world_body, r_world, R_mesh_to_body)
if nargin < 4 || isempty(R_mesh_to_body), R_mesh_to_body = eye(3); end
R = R_world_body * R_mesh_to_body;
T = eye(4); T(1:3,1:3) = R; T(1:3,4) = r_world(:);
if isgraphics(hT)                       % evita crashear si se borró
    set(hT, 'Matrix', T);
end
end
