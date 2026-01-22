function [hT, hMesh] = load_dron(ax, stl_path, escala_a_m, color_rgb)
% ax         : axes donde vivirán el hgtransform y el patch
% stl_path   : 'DRON EXPO v1.stl'
% escala_a_m : ej. 0.001 si el STL está en mm
% color_rgb  : [r g b] entre 0 y 1

if nargin < 3, escala_a_m = 0.001; end
if nargin < 4, color_rgb = [0.20 0.20 0.25]; end

tri = stlread(stl_path);  % devuelve triangulation o struct
if isa(tri,'triangulation')
    F = tri.ConnectivityList;
    V = tri.Points;
else
    % compatibilidad por si stlread devuelve struct
    if isfield(tri,'Faces'),     F = tri.Faces;     else, F = tri.faces;     end
    if isfield(tri,'Vertices'),  V = tri.Vertices;  else, V = tri.vertices;  end
end

V = V * escala_a_m;

hT = hgtransform('Parent', ax);  % PARENT = ax (clave para que no se "pierda")
hMesh = patch('Faces',F,'Vertices',V, ...
              'FaceColor',color_rgb,'EdgeColor','none', ...
              'Parent',hT);
end


