function wp = roadPath(centerline, sFrom, sTo, d, step)
%ROADPATH Waypoints that follow a curved road at a constant lateral offset.
%
%   COMPONENT STATUS: REAL
%
%   wp = ROADPATH(centerline, sFrom, sTo, d) returns waypoints from road
%   station sFrom to road station sTo along the scenario centreline, at
%   lateral offset d. If sTo < sFrom the path runs against the road's
%   direction (an oncoming road user). The offset d is always measured to
%   the LEFT of the road's own direction of increasing station, so an
%   oncoming vehicle keeping to its own left side (Indian driving) uses a
%   NEGATIVE d.
%
%   wp = ROADPATH(..., step) sets the waypoint spacing (default 2 m).
%
%   Why this exists (Phase 2)
%   -------------------------
%   The original scenarios gave actors two waypoints, one at each end of the
%   road. On curved roads the straight line between them ran up to 5 m off
%   the drivable surface, through walls and fields. Every actor on a curved
%   road now follows the road.
%
%   Inputs:
%       centerline - Kx2 scenario centreline
%       sFrom, sTo - road stations (m), clamped to the centreline length
%       d          - lateral offset (m), positive = left of road direction
%       step       - (optional) waypoint spacing (m)
%
%   Outputs:
%       wp - Mx2 waypoints, M >= 2
%
%   Example:
%       wp = roadPath(scn.centerline, 120, 0, -1.2);   % oncoming, own side
%
%   Requires: base MATLAB only.
%
%   See also MAKEACTOR, FRENETTOCARTESIAN.

if nargin < 5 || isempty(step), step = 2.0; end

sCl = pathArcLength(centerline);
L   = sCl(end);
sFrom = min(max(sFrom, 0), L);
sTo   = min(max(sTo,   0), L);

n  = max(2, ceil(abs(sTo - sFrom) / step) + 1);
sq = linspace(sFrom, sTo, n).';
wp = frenetToCartesian(centerline, sq, d);
end
