function [s, d, idx, foot] = projectPointOnPath(P, pt)
%PROJECTPOINTONPATH Project a point onto a polyline (Frenet coordinates).
%
%   COMPONENT STATUS: REAL
%
%   [s, d, idx, foot] = PROJECTPOINTONPATH(P, pt) finds the closest point on
%   polyline P to the query point pt, and returns its position in path
%   coordinates.
%
%   This is the bridge between world coordinates and the road-aligned frame
%   the IR-PSC planner works in. The lateral coordinate d is what lets the
%   planner say "this obstacle is 1.2 m to the left of my corridor centre"
%   without ever needing a lane marking.
%
%   Inputs:
%       P  - Nx2 polyline, N >= 2
%       pt - 1x2 query point [x y]
%
%   Outputs:
%       s    - arc length along P of the closest point (metres from P(1,:))
%       d    - signed lateral offset in metres. Positive = LEFT of the path
%              direction of travel, negative = RIGHT.
%       idx  - index of the segment start containing the projection
%       foot - 1x2 coordinates of the closest point itself
%
%   The search is exhaustive over segments. For the path lengths used here
%   (tens of points) that is both fast enough and immune to the local-minimum
%   traps a gradient search would hit on a curved corridor.
%
%   Example:
%       [s, d] = projectPointOnPath([0 0; 10 0], [4 2]);   % s = 4, d = 2
%
%   Requires: base MATLAB only.
%
%   See also PATHARCLENGTH, FRENETTOCARTESIAN.

requireInput(isnumeric(P) && size(P,2) == 2 && all(isfinite(P(:))), 'projectPointOnPath', 'P must be a finite Nx2');
requireInput(numel(pt) == 2 && all(isfinite(pt)), 'projectPointOnPath', 'pt must be a finite 1x2');
pt = pt(:).';

N = size(P,1);
if N < 2
    error('projectPointOnPath:tooShort', 'Need at least 2 path points.');
end

% Vectorised over all segments (Phase 2, for speed; identical result).
A  = P(1:N-1, :);
AB = P(2:N, :) - A;
L2 = AB(:,1).^2 + AB(:,2).^2;
AP = [pt(1) - A(:,1), pt(2) - A(:,2)];
t  = (AP(:,1) .* AB(:,1) + AP(:,2) .* AB(:,2)) ./ max(L2, 1e-12);
t(L2 < 1e-12) = 0;
t  = min(max(t, 0), 1);                    % clamp to each segment
F  = A + [t .* AB(:,1), t .* AB(:,2)];
d2 = (pt(1) - F(:,1)).^2 + (pt(2) - F(:,2)).^2;

[bestD2, idx] = min(d2);                   % first minimum, as the loop did
foot = F(idx, :);
segLen = sqrt(L2);
sCum = [0; cumsum(segLen)];
s = sCum(idx) + t(idx) * segLen(idx);

% Sign convention: z-component of cross(tangent, a->pt).
% Positive means pt lies to the LEFT of the direction of travel.
bestSide = AB(idx,1) * AP(idx,2) - AB(idx,2) * AP(idx,1);
d = sqrt(bestD2) * sign(bestSide);
if bestSide == 0
    d = 0;
end
end
