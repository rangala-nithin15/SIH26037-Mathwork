function hit = potholeWheelOverlap(corridor, D, potholes, cfg, vp, margin)
%POTHOLEWHEELOVERLAP Which (station, offset) choices put a tyre into a pothole.
%
%   COMPONENT STATUS: SIMPLIFIED (planar wheel-track geometry)
%
%   hit = POTHOLEWHEELOVERLAP(corridor, D, potholes, cfg, vp) returns an
%   NxKxP logical array. hit(i,j,p) is true when a vehicle whose rear axle
%   passes corridor station i at lateral offset D(i,j) would run a tyre
%   (rear OR front axle, left OR right wheel) through pothole p.
%
%   Why wheel tracks and not the body footprint
%   -------------------------------------------
%   A pothole is a road-surface hazard, not an obstacle: the body passes
%   over it harmlessly, only the tyres drop in. A small pothole that fits
%   between the two wheel tracks can be STRADDLED at full speed, which is
%   exactly what an experienced Indian driver does. Treating potholes as
%   occupied cells would forbid that and would wrongly block narrow roads.
%
%   Geometry: each pothole is an ellipse (length along its yaw, width
%   across) projected into the corridor frame as an along/across extent.
%   Each wheel is a strip of width cfg.pothole.tyreWidth centred at lateral
%   offset d +/- wheelTrack/2. The rear wheels are at the station, the front
%   wheels one wheelbase further along.
%
%   Inputs:
%       corridor - corridor struct from extractCorridor()
%       D        - NxK lateral offsets (m), or Nx1 for a single path
%       potholes - struct array with fields .pos .length .width .yaw
%                  (pothole tracks from potholeTracker, or ground truth)
%       cfg      - config struct from irpscConfig()
%       vp       - vehicle params struct from vehicleParams()
%       margin   - (optional) extra lateral clearance (m) added around each
%                  pothole. Default 0. Planning uses
%                  cfg.pothole.lateralMargin so the controller's small
%                  tracking error does not put a tyre on the rim.
%
%   Outputs:
%       hit - NxKxP logical
%
%   Requires: base MATLAB only.
%
%   See also POTHOLECOSTGRID, POTHOLETRACKER, IRPSCPLANNER.

if nargin < 6 || isempty(margin), margin = 0; end
N = size(corridor.center, 1);
if size(D,1) ~= N
    D = D(:);
end
K = size(D, 2);
P = numel(potholes);
hit = false(N, K, P);
if P == 0 || N < 2
    return;
end

halfTrack = cfg.pothole.wheelTrack / 2;
halfTyre  = cfg.pothole.tyreWidth / 2 + margin;
s = corridor.s(:);

for p = 1:P
    ph = potholes(p);
    [sp, dp, idx] = projectPointOnPath(corridor.center, ph.pos);
    % Ignore potholes the projection clamps beyond either end of the corridor.
    if sp <= 1e-6 && norm(ph.pos - corridor.center(1,:)) > ph.length
        continue;
    end
    if sp >= s(end) - 1e-6 && norm(ph.pos - corridor.center(end,:)) > ph.length
        continue;
    end
    rel = ph.yaw - corridor.heading(min(idx, N));
    halfAlong  = sqrt((ph.length/2 * cos(rel))^2 + (ph.width/2 * sin(rel))^2);
    halfAcross = sqrt((ph.length/2 * sin(rel))^2 + (ph.width/2 * cos(rel))^2);

    % Rear wheels at station s, front wheels at s + wheelbase.
    rearIn  = abs(s - sp) <= halfAlong;
    frontIn = abs(s + vp.wheelbase - sp) <= halfAlong;
    rows = find(rearIn | frontIn);
    if isempty(rows), continue; end

    for i = rows(:).'
        d = D(i, :);
        leftW  = abs(d + halfTrack - dp) <= halfAcross + halfTyre;
        rightW = abs(d - halfTrack - dp) <= halfAcross + halfTyre;
        hit(i, :, p) = leftW | rightW;
    end
end
end
