function [C, info] = potholeCostGrid(corridor, offsets, potholes, cfg, vp)
%POTHOLECOSTGRID Traversal cost of each (station, offset) cell due to potholes.
%
%   COMPONENT STATUS: REAL (cost model) over SIMPLIFIED geometry
%
%   [C, info] = POTHOLECOSTGRID(corridor, offsets, potholes, cfg, vp) builds
%   the NxK cost matrix that DEFORMTRAJECTORY adds to its stage cost, so the
%   same dynamic programme that bends around predicted road users also
%   decides whether to steer around a pothole or drive through it.
%
%   Cost of a cell = sum over potholes whose tyre strip it enters of
%       cfg.pothole.cost.<severity> * detection confidence
%
%   The trade-off is therefore explicit and tunable: a MINOR pothole
%   (cost 0.6) is not worth a large swerve and is usually straddled or
%   driven over slowly; a MODERATE one (3.0) is avoided when there is
%   room; a SEVERE one (12.0) is avoided unless every alternative leaves the
%   road or enters predicted traffic, in which case it is crossed at crawl
%   speed (see IRPSCPLANNER, which applies cfg.pothole.speed).
%
%   The cost is kept separate from the collision-risk matrix R on purpose:
%   a pothole must change the path and the speed, but it must never be
%   reported as collision risk or trigger an emergency stop by itself.
%
%   Inputs:
%       corridor - corridor struct from extractCorridor()
%       offsets  - 1xK lateral offsets of the DP grid (m)
%       potholes - CONFIRMED pothole tracks (struct array), may be empty
%       cfg, vp  - config and vehicle params
%
%   Outputs:
%       C    - NxK non-negative cost
%       info - struct: .hit (NxKxP logical), .unitCost (1xP)
%
%   Requires: base MATLAB only.
%
%   See also POTHOLEWHEELOVERLAP, DEFORMTRAJECTORY, POTHOLETRACKER.

N = size(corridor.center, 1);
K = numel(offsets);
C = zeros(N, K);
info.hit      = false(N, K, 0);
info.unitCost = zeros(1, 0);
if isempty(potholes) || N < 2
    return;
end

D   = repmat(offsets(:).', N, 1);
hit = potholeWheelOverlap(corridor, D, potholes, cfg, vp, cfg.pothole.lateralMargin);

P = numel(potholes);
unit = zeros(1, P);
for p = 1:P
    unit(p) = severityValue(cfg.pothole.cost, potholes(p).severity) * ...
              max(min(potholes(p).confidence, 1), 0);
    C = C + unit(p) * double(hit(:,:,p));
end
info.hit      = hit;
info.unitCost = unit;
end

function v = severityValue(table, sev)
if isfield(table, sev)
    v = table.(sev);
else
    v = table.moderate;
end
end
