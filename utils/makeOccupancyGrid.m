function grid = makeOccupancyGrid(nRows, nCols, resolution, origin)
%MAKEOCCUPANCYGRID Create an empty drivable-space occupancy grid.
%
%   COMPONENT STATUS: REAL
%
%   grid = MAKEOCCUPANCYGRID(nRows, nCols, resolution, origin) builds the
%   grid structure that represents drivable space in this project.
%
%   Drivable space is the PRIMARY planning constraint in IR-PSC, so this
%   structure is the planner's most important input. It deliberately does
%   NOT encode lanes.
%
%   Inputs:
%       nRows      - integer, grid rows (y direction)
%       nCols      - integer, grid columns (x direction)
%       resolution - metres per cell (e.g. 0.20)
%       origin     - 1x2 [x y] world coordinates of the CENTRE of cell (1,1)
%
%   Outputs:
%       grid - struct with fields:
%           .occ        nRows x nCols logical. true = OCCUPIED / not drivable
%           .resolution metres per cell
%           .origin     1x2 world coordinates of cell (1,1) centre
%           .nRows, .nCols
%
%   Convention: occ(r,c) covers world x in origin(1) + (c-1)*res +/- res/2
%   and world y in origin(2) + (r-1)*res +/- res/2. Row index maps to y,
%   column index maps to x.
%
%   Cells outside the grid are treated as OCCUPIED by rayCastGrid and
%   isOccupiedAt, i.e. unknown space is not drivable. That is the safe
%   direction to err.
%
%   Example:
%       g = makeOccupancyGrid(200, 400, 0.25, [-10 -25]);
%
%   Requires: base MATLAB only.
%
%   See also ISOCCUPIEDAT, RAYCASTGRID, EXTRACTCORRIDOR.

validateattributes(nRows, {'numeric'}, {'scalar','integer','positive'}, mfilename, 'nRows');
validateattributes(nCols, {'numeric'}, {'scalar','integer','positive'}, mfilename, 'nCols');
validateattributes(resolution, {'numeric'}, {'scalar','positive','finite'}, mfilename, 'resolution');
validateattributes(origin, {'numeric'}, {'vector','numel',2,'finite','real'}, mfilename, 'origin');

grid.occ        = false(nRows, nCols);
grid.resolution = resolution;
grid.origin     = origin(:).';
grid.nRows      = nRows;
grid.nCols      = nCols;
end
