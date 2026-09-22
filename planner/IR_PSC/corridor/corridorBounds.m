function [dMin, dMax, marginUsed] = corridorBounds(corridor, egoHalfWidth, margin, minMargin)
%CORRIDORBOUNDS Allowed lateral-offset band at each corridor station.
%
%   COMPONENT STATUS: REAL
%
%   [dMin, dMax] = CORRIDORBOUNDS(corridor, egoHalfWidth, margin)
%   [dMin, dMax, marginUsed] = CORRIDORBOUNDS(corridor, egoHalfWidth, margin, minMargin)
%   returns, for every station, how far the trajectory may move left (dMax)
%   and right (dMin) from the corridor centreline while keeping the whole
%   vehicle body inside the drivable corridor.
%
%   IR-PSC step 8: preserve road-boundary safety margins. This is the
%   constraint that the deformation stage is not permitted to violate --
%   dodging an obstacle must never push the vehicle off the road.
%
%   The allowed band at station i is
%
%       need - rightDist(i)  <=  d  <=  leftDist(i) - need,
%       need = egoHalfWidth + margin(i)
%
%   Narrow stations (Phase 2)
%   -------------------------
%   Where the corridor is narrower than the vehicle plus the preferred
%   margin, the margin shrinks towards minMargin (never below it), so the
%   vehicle may pass a narrow gap slowly with smaller -- but still
%   non-negative and configured -- side clearance. The planner applies a
%   speed cap at such stations. Where even minMargin does not fit, the band
%   collapses to the single best-centred offset; such a station is BLOCKED
%   in extractCorridor and the planner stops before it.
%
%   The earlier version clamped the band to always include d = 0, even when
%   the (temporally blended) centreline was not centred and d = 0 did not
%   keep the body on the road. The band is now computed exactly.
%
%   Inputs:
%       corridor     - struct from extractCorridor()
%       egoHalfWidth - m, half the ego vehicle width
%       margin       - m, preferred clearance from each boundary
%       minMargin    - (optional) m, smallest margin allowed at narrow
%                      stations. Default: margin (no shrinking).
%
%   Outputs:
%       dMin       - Nx1, most negative (rightward) offset allowed
%       dMax       - Nx1, most positive (leftward) offset allowed, dMax >= dMin
%       marginUsed - Nx1, the side margin actually applied at each station
%
%   Example:
%       [dMin, dMax] = corridorBounds(corr, 0.84, 0.35, 0.10);
%
%   Requires: base MATLAB only.
%
%   See also EXTRACTCORRIDOR, DEFORMTRAJECTORY, CHECKCLEARANCE.

if nargin < 4 || isempty(minMargin), minMargin = margin; end

validateattributes(egoHalfWidth, {'numeric'}, {'scalar','positive','finite'}, ...
                   mfilename, 'egoHalfWidth');
validateattributes(margin, {'numeric'}, {'scalar','nonnegative','finite'}, ...
                   mfilename, 'margin');
validateattributes(minMargin, {'numeric'}, {'scalar','nonnegative','finite'}, ...
                   mfilename, 'minMargin');

l = corridor.leftDist(:);
r = corridor.rightDist(:);
w = l + r;

% Largest margin <= preferred that still leaves a non-empty band.
fitMargin  = (w - 2 * egoHalfWidth) / 2;
marginUsed = min(margin, max(fitMargin, minMargin));

need = egoHalfWidth + marginUsed;
dMax = l - need;
dMin = need - r;

inverted = dMin > dMax;
mid = (l - r) / 2;                    % best-centred offset
dMax(inverted) = mid(inverted);
dMin(inverted) = mid(inverted);
end
