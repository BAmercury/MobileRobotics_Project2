function map = loadMap(filename, res_xy, res_z, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, res_xy, res_z).  Creates an occupancy grid map
%  which represents obstacles and .
map = BinaryOccupancyGrid3D.LoadMap(filename, res_xy, res_z, margin);
end
