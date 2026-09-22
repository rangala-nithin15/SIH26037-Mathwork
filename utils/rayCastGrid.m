function [dist, hitPt] = rayCastGrid(grid, origin, heading, maxRange, step)
%RAYCASTGRID March a ray through the occupancy grid until it hits something.
%
%   COMPONENT STATUS: REAL
%
%   [dist, hitPt] = RAYCASTGRID(grid, origin, heading, maxRange, step)
%   advances from `origin` along `heading` in increments of `step` until an
%   occupied cell (or the grid edge) is reached.
%
%   This is the primitive that turns raw drivable space into corridor
%   boundaries: cast a ray left and right from a station on the reference
%   path and the two hits ARE the road edges. No lane detection involved,
%   which is precisely the point of IR-PSC.
%
%   Fixed-step marching is used rather than an exact DDA traversal because
%   it is simple, dependency-free and easy to unit test. With step <= half a
%   cell it cannot skip an occupied cell.
%
%   Inputs:
%       grid     - struct from makeOccupancyGrid()
%       origin   - 1x2 [x y] start point in world coordinates
%       heading  - scalar, ray direction in radians
%       maxRange - scalar, maximum distance to search (m)
%       step     - scalar, march increment (m). Should be <= resolution/2.
%
%   Outputs:
%       dist  - distance to the first occupied sample, or maxRange if the ray
%               stayed clear for its whole length
%       hitPt - 1x2 world coordinates of the hit (or of the ray end)
%
%   If the origin itself is occupied, dist = 0 is returned.
%
%   Example:
%       [d, p] = rayCastGrid(g, [0 0], pi/2, 12, 0.1);
%
%   Requires: base MATLAB only.
%
%   See also ISOCCUPIEDAT, EXTRACTCORRIDOR.

requireInput(numel(origin) == 2 && all(isfinite(origin)), 'rayCastGrid', 'origin must be a finite 1x2');
requireInput(isscalar(heading) && isfinite(heading), 'rayCastGrid', 'heading must be a finite scalar');
requireInput(isscalar(maxRange) && maxRange > 0 && isfinite(maxRange), 'rayCastGrid', 'maxRange must be positive');
requireInput(isscalar(step) && step > 0 && isfinite(step), 'rayCastGrid', 'step must be positive');

origin = origin(:).';
dirVec = [cos(heading), sin(heading)];

nSteps = max(1, ceil(maxRange / step));
ds     = (0:nSteps).' * step;
ds(ds > maxRange) = maxRange;

samples = origin + ds .* dirVec;
occ     = isOccupiedAt(grid, samples);

first = find(occ, 1, 'first');
if isempty(first)
    dist  = maxRange;
    hitPt = origin + maxRange * dirVec;
else
    dist  = ds(first);
    hitPt = samples(first, :);
end
end
