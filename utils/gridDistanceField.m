function grid = gridDistanceField(grid, maxDist)
%GRIDDISTANCEFIELD Attach a capped distance-to-nearest-obstacle field to a grid.
%
%   COMPONENT STATUS: REAL (discretised Euclidean distance, capped)
%
%   grid = GRIDDISTANCEFIELD(grid, maxDist) adds grid.dist, an nRows x nCols
%   array holding, for every cell, the distance in metres from its centre to
%   the centre of the nearest OCCUPIED cell, capped at maxDist, and
%   grid.distMax = maxDist.
%
%   Method: occupied cells are dilated by discs of increasing radius using
%   conv2 (base MATLAB). A cell's distance is the smallest radius whose
%   dilation reaches it, so the result is exact to one radius increment
%   (grid.resolution) up to maxDist. It is computed ONCE per static grid.
%
%   Why (Phase 2): the deformation stage only saw static obstacles through
%   the corridor boundary rays cast perpendicular at each station, so the
%   corner of a stall or a parked truck between two rays was invisible to
%   it. The planner would choose a path, the exact clearance check would
%   reject it, and the vehicle stopped in front of a static obstacle for
%   good. With this field the dynamic programme scores static clearance for
%   every (station, offset) cell directly.
%
%   Inputs:
%       grid    - struct from makeOccupancyGrid()
%       maxDist - m, cap (default 3.0)
%
%   Outputs:
%       grid - same struct with .dist and .distMax
%
%   Requires: base MATLAB only.
%
%   See also STATICCLEARANCECOST, BUILDROADGRID.

if nargin < 2 || isempty(maxDist), maxDist = 3.0; end

res   = grid.resolution;
occ   = double(grid.occ);
nStep = max(1, ceil(maxDist / res));
dist  = repmat(maxDist, grid.nRows, grid.nCols);
dist(grid.occ) = 0;
unset = ~grid.occ;

for k = 1:nStep
    r = k * res;
    [xx, yy] = meshgrid(-k:k, -k:k);
    kernel = double(xx.^2 + yy.^2 <= k^2 + 1e-9);
    reached = conv2(occ, kernel, 'same') > 0;
    newly = unset & reached;
    dist(newly) = min(r, maxDist);
    unset = unset & ~reached;
    if ~any(unset(:))
        break;
    end
end

grid.dist    = dist;
grid.distMax = maxDist;
end
