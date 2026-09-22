function grid = buildRoadGrid(centerline, halfWidth, resolution, extras)
%BUILDROADGRID Rasterise a road corridor into a drivable-space grid.
%
%   COMPONENT STATUS: REAL
%
%   grid = BUILDROADGRID(centerline, halfWidth, resolution, extras) builds
%   the occupancy grid the planner treats as drivable space.
%
%   Everything starts OCCUPIED, and the road is then carved out as free
%   space. That ordering is deliberate: it means anything the scenario did
%   not explicitly declare drivable is not drivable, so an error of omission
%   produces over-caution rather than a phantom road.
%
%   The road is defined by a centreline and a half-width that may VARY along
%   it. Varying width is what makes these scenarios Indian-road-like: the
%   drivable surface narrows and widens irregularly, which is precisely the
%   situation a fixed-lane-offset planner handles badly and a corridor-based
%   planner handles naturally.
%
%   No lane markings are represented anywhere in this structure. There is
%   nothing for a lane-based planner to find, by design.
%
%   Inputs:
%       centerline - Kx2 road centreline points
%       halfWidth  - scalar, or Kx1 half-width at each centreline point (m)
%       resolution - metres per cell (e.g. 0.20)
%       extras     - (optional) struct array of obstacles to mark as
%                    occupied, each with fields:
%                      .center 1x2, .radius scalar   (circular), or
%                      .center 1x2, .halfSize 1x2, .yaw   (rectangular)
%                    Used for parked vehicles, buildings, road-edge debris
%                    and potholes.
%
%   Outputs:
%       grid - occupancy grid struct in makeOccupancyGrid() format
%
%   Example:
%       g = buildRoadGrid(cl, 3.2, 0.2);
%
%   Requires: base MATLAB only.
%
%   See also MAKEOCCUPANCYGRID, EXTRACTCORRIDOR.

if nargin < 4, extras = []; end

validateattributes(centerline, {'numeric'}, {'2d','ncols',2,'finite','real'}, ...
                   mfilename, 'centerline');

K = size(centerline,1);
if isscalar(halfWidth)
    halfWidth = repmat(halfWidth, K, 1);
end
halfWidth = halfWidth(:);

% --- Grid extent, with a margin so rays can run off the road ------------
margin = max(halfWidth) + 6;
xMin = min(centerline(:,1)) - margin;
xMax = max(centerline(:,1)) + margin;
yMin = min(centerline(:,2)) - margin;
yMax = max(centerline(:,2)) + margin;

nCols = max(2, ceil((xMax - xMin) / resolution) + 1);
nRows = max(2, ceil((yMax - yMin) / resolution) + 1);

grid = makeOccupancyGrid(nRows, nCols, resolution, [xMin, yMin]);
grid.occ(:) = true;                       % everything occupied to start

% --- Carve the road as free space --------------------------------------
% The centreline is densified first so the swept discs overlap and leave no
% occupied speckles inside the road.
sCl     = pathArcLength(centerline);
denseN  = max(K, ceil(sCl(end) / (resolution/2)) + 1);
clDense = resamplePath(centerline, denseN);
hwDense = interp1(linspace(0,1,K).', halfWidth, linspace(0,1,denseN).', 'linear');

[cols, rows] = meshgrid(1:nCols, 1:nRows);
X = grid.origin(1) + (cols - 1) * resolution;
Y = grid.origin(2) + (rows - 1) * resolution;

free = false(nRows, nCols);
for i = 1:denseN
    % Only cells inside this disc's bounding box can change (Phase 2, speed).
    c0 = max(1, floor((clDense(i,1) - hwDense(i) - grid.origin(1)) / resolution) + 1);
    c1 = min(nCols, ceil((clDense(i,1) + hwDense(i) - grid.origin(1)) / resolution) + 1);
    r0 = max(1, floor((clDense(i,2) - hwDense(i) - grid.origin(2)) / resolution) + 1);
    r1 = min(nRows, ceil((clDense(i,2) + hwDense(i) - grid.origin(2)) / resolution) + 1);
    if c0 > c1 || r0 > r1, continue; end
    Xs = X(r0:r1, c0:c1);  Ys = Y(r0:r1, c0:c1);
    d2 = (Xs - clDense(i,1)).^2 + (Ys - clDense(i,2)).^2;
    free(r0:r1, c0:c1) = free(r0:r1, c0:c1) | (d2 <= hwDense(i)^2);
end
grid.occ(free) = false;

% --- Stamp extra obstacles back in as occupied -------------------------
for e = 1:numel(extras)
    ex = extras(e);
    if isfield(ex, 'radius') && ~isempty(ex.radius)
        d2 = (X - ex.center(1)).^2 + (Y - ex.center(2)).^2;
        grid.occ(d2 <= ex.radius^2) = true;
    elseif isfield(ex, 'halfSize') && ~isempty(ex.halfSize)
        yaw = 0;
        if isfield(ex, 'yaw') && ~isempty(ex.yaw)
            yaw = ex.yaw;
        end
        dx = X - ex.center(1);
        dy = Y - ex.center(2);
        xr =  dx*cos(yaw) + dy*sin(yaw);
        yr = -dx*sin(yaw) + dy*cos(yaw);
        inside = abs(xr) <= ex.halfSize(1) & abs(yr) <= ex.halfSize(2);
        grid.occ(inside) = true;
    end
end

% Distance-to-obstacle field for the planner's static clearance cost
% (Phase 2). Computed once, here, because the grid is static.
grid = gridDistanceField(grid, 3.0);
end
