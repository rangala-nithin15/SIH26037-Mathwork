function center = extractCenterline(leftPt, rightPt, smoothHalfWin)
%EXTRACTCENTERLINE Continuous centreline midway between corridor boundaries.
%
%   COMPONENT STATUS: REAL
%
%   center = EXTRACTCENTERLINE(leftPt, rightPt, smoothHalfWin) returns the
%   midpoint path between the two boundaries, smoothed.
%
%   This is IR-PSC step 7 in its simplest form: a CONTINUOUS road-following
%   preferred path. Note what it is not -- it is not one of five fixed lane
%   candidates. The corridor centre moves continuously as the road narrows,
%   widens or bends, which is the behaviour an unmarked Indian road needs.
%
%   Inputs:
%       leftPt        - Nx2 left boundary points
%       rightPt       - Nx2 right boundary points (same N)
%       smoothHalfWin - moving-average half-window; 0 disables smoothing
%
%   Outputs:
%       center - Nx2 smoothed centreline
%
%   The first point is NOT locked here, because the corridor centre near the
%   vehicle is a measurement rather than a commanded start point. The
%   trajectory generator anchors the path to the ego position later.
%
%   Example:
%       center = extractCenterline(L, R, 5);
%
%   Requires: base MATLAB only.
%
%   See also EXTRACTCORRIDOR, SMOOTHPATH.

validateattributes(leftPt,  {'numeric'}, {'2d','ncols',2,'finite','real'}, mfilename, 'leftPt');
validateattributes(rightPt, {'numeric'}, {'2d','ncols',2,'finite','real'}, mfilename, 'rightPt');

if size(leftPt,1) ~= size(rightPt,1)
    error('extractCenterline:sizeMismatch', ...
          'leftPt has %d rows but rightPt has %d.', size(leftPt,1), size(rightPt,1));
end

center = 0.5 * (leftPt + rightPt);

if nargin >= 3 && ~isempty(smoothHalfWin) && smoothHalfWin > 0 && size(center,1) >= 3
    center = smoothPath(center, smoothHalfWin, 2, false);
end
end
