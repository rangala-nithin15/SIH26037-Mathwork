function [C, clr] = staticClearanceCost(corridor, offsets, grid, cfg, vp)
%STATICCLEARANCECOST Static-obstacle clearance of each (station, offset) cell.
%
%   COMPONENT STATUS: REAL
%
%   [C, clr] = STATICCLEARANCECOST(corridor, offsets, grid, cfg, vp)
%   evaluates, for a vehicle whose REAR AXLE is at corridor station i with
%   lateral offset offsets(j) and heading along the corridor, the clearance
%   between its body outline and the nearest occupied grid cell, using the
%   precomputed distance field (GRIDDISTANCEFIELD).
%
%   The returned cost matrix for DEFORMTRAJECTORY is
%       Inf                                  if clr < cfg.safety.minLateralClearance
%       wStatic * (lateralClearance - clr)^2 if clr < cfg.safety.lateralClearance
%       0                                    otherwise
%   so the path is pushed away from stalls, parked vehicles and road edges
%   towards the preferred clearance, and is never allowed closer than the
%   hard minimum -- the same rule CHECKCLEARANCE applies afterwards.
%
%   The body outline is sampled at the corners and along the edges at
%   ~0.5 m spacing. Clearance is accurate to about one grid cell; the exact
%   rectangle test in CHECKCLEARANCE remains the final authority.
%
%   Inputs:
%       corridor - corridor struct (usable part)
%       offsets  - 1xK lateral offsets (m)
%       grid     - occupancy grid; .dist is computed here if absent
%       cfg, vp  - config and vehicle params
%
%   Outputs:
%       C   - NxK cost (Inf = forbidden)
%       clr - NxK clearance (m), capped at the distance-field maximum
%
%   Requires: base MATLAB only.
%
%   See also GRIDDISTANCEFIELD, DEFORMTRAJECTORY, CHECKCLEARANCE.

if ~isfield(grid, 'dist')
    grid = gridDistanceField(grid, cfg.safety.clearanceSearch);
end

N = size(corridor.center, 1);
K = numel(offsets);

% Body outline sample points in the body frame (x forward from rear axle).
xs = linspace(-vp.rearOverhang, vp.frontOverhang, max(3, ceil(vp.length / 0.5) + 1));
ys = linspace(-vp.halfWidth, vp.halfWidth, max(3, ceil(vp.width / 0.5) + 1));
bx = [xs, xs, repmat(xs(1), 1, numel(ys)), repmat(xs(end), 1, numel(ys))];
by = [repmat(ys(1), 1, numel(xs)), repmat(ys(end), 1, numel(xs)), ys, ys];
nb = numel(bx);

% Rear-axle positions for every (station, offset).
sAll = repmat(corridor.s(:), K, 1);
dAll = reshape(repmat(offsets(:).', N, 1), [], 1);
P    = frenetToCartesian(corridor.center, sAll, dAll);          % (N*K)x2
th   = repmat(corridor.heading(:), K, 1);
ct = cos(th);  st = sin(th);

X = P(:,1) + ct * bx - st * by;                                 % (N*K) x nb
Y = P(:,2) + st * bx + ct * by;

res = grid.resolution;
cc = round((X - grid.origin(1)) / res) + 1;
rr = round((Y - grid.origin(2)) / res) + 1;
inside = rr >= 1 & rr <= grid.nRows & cc >= 1 & cc <= grid.nCols;
d = zeros(size(X));                                             % outside grid = 0 clearance
idx = sub2ind([grid.nRows, grid.nCols], rr(inside), cc(inside));
d(inside) = grid.dist(idx);

% Subtract a further grid cell: the sampled outline and cell-centre
% rounding can overestimate clearance by up to about one cell relative to
% the exact rectangle test in CHECKCLEARANCE, and the planner must be at
% least as strict as the check that follows it.
clr = min(d, [], 2) - res / 2 - res;
clr = reshape(clr, N, K);

C = zeros(N, K);
soft = clr < cfg.safety.lateralClearance;
C(soft) = cfg.deform.wStatic * (cfg.safety.lateralClearance - clr(soft)).^2;
% Hard limit beyond the first metre only: the vehicle is where it is, and a
% current pose that is already tight must not make every plan infeasible.
hard = clr < cfg.safety.minLateralClearance;
hard(corridor.s(:) <= 1.0, :) = false;
C(hard) = Inf;
end
