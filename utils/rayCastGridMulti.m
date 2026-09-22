function dist = rayCastGridMulti(grid, origins, headings, maxRange, step)
%RAYCASTGRIDMULTI Cast many rays through the occupancy grid in one call.
%
%   COMPONENT STATUS: REAL
%
%   dist = RAYCASTGRIDMULTI(grid, origins, headings, maxRange, step) returns
%   for each ray j the distance to the first occupied sample, exactly as
%   RAYCASTGRID would for origins(j,:) and headings(j), but with all rays
%   sampled in a single vectorised occupancy query.
%
%   Added in Phase 2 purely for speed: corridor extraction casts about a
%   thousand rays per planning cycle, and calling RAYCASTGRID one ray at a
%   time dominated the planner's run time.
%
%   Inputs:
%       grid     - struct from makeOccupancyGrid()
%       origins  - Rx2 ray start points (or 1x2, shared by all rays)
%       headings - Rx1 ray directions (rad)
%       maxRange - scalar maximum distance (m)
%       step     - scalar march increment (m)
%
%   Outputs:
%       dist - Rx1 distances; maxRange where a ray stays clear
%
%   Requires: base MATLAB only.
%
%   See also RAYCASTGRID, SEEDCENTERLINE, EXTRACTCORRIDOR.

headings = headings(:);
R = numel(headings);
if size(origins,1) == 1
    origins = repmat(origins, R, 1);
end

nSteps = max(1, ceil(maxRange / step));
ds     = (0:nSteps) * step;
ds(ds > maxRange) = maxRange;
S = numel(ds);

X = origins(:,1) + cos(headings) * ds;          % R x S
Y = origins(:,2) + sin(headings) * ds;

occ = isOccupiedAt(grid, [X(:), Y(:)]);
occ = reshape(occ, R, S);

dist = repmat(maxRange, R, 1);
anyHit = any(occ, 2);
if any(anyHit)
    [~, firstIdx] = max(occ(anyHit, :), [], 2);  % first true in each row
    dist(anyHit) = ds(firstIdx).';
end
end
