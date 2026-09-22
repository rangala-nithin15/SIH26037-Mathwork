function occupied = isOccupiedAt(grid, pts)
%ISOCCUPIEDAT Query occupancy of the grid at world coordinates.
%
%   COMPONENT STATUS: REAL
%
%   occupied = ISOCCUPIEDAT(grid, pts) returns true for each query point
%   that falls in an occupied cell OR outside the grid.
%
%   Treating out-of-grid as occupied is deliberate: space we have no
%   information about is not space we are willing to drive into. This makes
%   the planner fail safe at the edge of sensor coverage instead of
%   confidently planning into the unknown.
%
%   Inputs:
%       grid - struct from makeOccupancyGrid()
%       pts  - Mx2 array of [x y] world points
%
%   Outputs:
%       occupied - Mx1 logical
%
%   Example:
%       occ = isOccupiedAt(g, [1.0 2.0; 3.0 4.0]);
%
%   Requires: base MATLAB only.
%
%   See also MAKEOCCUPANCYGRID, RAYCASTGRID.

requireInput(isnumeric(pts) && size(pts,2) == 2, 'isOccupiedAt', 'pts must be Mx2');

M = size(pts,1);
occupied = true(M,1);          % default: unknown => occupied

c = round((pts(:,1) - grid.origin(1)) / grid.resolution) + 1;
r = round((pts(:,2) - grid.origin(2)) / grid.resolution) + 1;

inside = r >= 1 & r <= grid.nRows & c >= 1 & c <= grid.nCols & ...
         isfinite(r) & isfinite(c);

if any(inside)
    linIdx = sub2ind([grid.nRows, grid.nCols], r(inside), c(inside));
    occupied(inside) = grid.occ(linIdx);
end
end
