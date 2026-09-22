function Q = frenetToCartesian(P, sQuery, dQuery)
%FRENETTOCARTESIAN Convert road-aligned (s,d) coordinates back to world x,y.
%
%   COMPONENT STATUS: REAL
%
%   Q = FRENETTOCARTESIAN(P, sQuery, dQuery) places points at arc length
%   sQuery along the reference polyline P, offset laterally by dQuery
%   (positive = LEFT of the direction of travel).
%
%   This is the inverse of PROJECTPOINTONPATH and is how the deformed
%   lateral-offset profile produced by the IR-PSC planner is turned back into
%   a drivable world-frame trajectory.
%
%   Inputs:
%       P      - Nx2 reference polyline, N >= 2
%       sQuery - Mx1 arc lengths along P (clamped to [0, totalLength])
%       dQuery - Mx1 lateral offsets in metres, or a scalar applied to all
%
%   Outputs:
%       Q - Mx2 world-frame points
%
%   Example:
%       Q = frenetToCartesian([0 0; 10 0], [0; 5; 10], 1.0);
%       % -> [0 1; 5 1; 10 1]
%
%   Requires: base MATLAB only.
%
%   See also PROJECTPOINTONPATH, DEFORMTRAJECTORY.

requireInput(isnumeric(P) && size(P,2) == 2 && all(isfinite(P(:))), 'frenetToCartesian', 'P must be a finite Nx2');
requireInput(isnumeric(sQuery) && isvector(sQuery) && all(isfinite(sQuery)), 'frenetToCartesian', 'sQuery must be a finite vector');
requireInput(isnumeric(dQuery) && all(isfinite(dQuery(:))), 'frenetToCartesian', 'dQuery must be finite');

sQuery = sQuery(:);
M      = numel(sQuery);
if isscalar(dQuery)
    dQuery = repmat(dQuery, M, 1);
else
    dQuery = dQuery(:);
    if numel(dQuery) ~= M
        error('frenetToCartesian:sizeMismatch', ...
              'dQuery must be scalar or the same length as sQuery.');
    end
end

s  = pathArcLength(P);
th = pathHeading(P);

% Strictly increasing stations only, so interp1 is well posed.
keep = [true; diff(s) > 1e-9];
sU   = s(keep);
xU   = P(keep,1);
yU   = P(keep,2);
thU  = th(keep);

if numel(sU) < 2
    Q = repmat(P(1,:), M, 1);
    return;
end

sc = min(max(sQuery, 0), sU(end));      % clamp rather than extrapolate

xc = interp1(sU, xU, sc, 'linear');
yc = interp1(sU, yU, sc, 'linear');

% Interpolate heading through its unit vector so the +pi/-pi seam does not
% produce a spurious sweep through zero.
cx = interp1(sU, cos(thU), sc, 'linear');
cy = interp1(sU, sin(thU), sc, 'linear');
thc = atan2(cy, cx);

% Left normal of the tangent is (-sin, cos).
Q = [xc - dQuery .* sin(thc), ...
     yc + dQuery .* cos(thc)];
end
