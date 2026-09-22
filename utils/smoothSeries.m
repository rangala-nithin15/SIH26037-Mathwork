function y = smoothSeries(x, halfWin, passes)
%SMOOTHSERIES Moving-average smoothing of a scalar series (shrinking window).
%
%   COMPONENT STATUS: REAL
%
%   y = SMOOTHSERIES(x, halfWin, passes) smooths the column vector x. Used
%   for lateral-offset profiles and speed profiles, where SMOOTHPATH would
%   be the wrong shape.
%
%   Inputs:
%       x       - Nx1 numeric vector
%       halfWin - non-negative integer half-window (0 = no smoothing)
%       passes  - positive integer number of passes (default 1)
%
%   Outputs:
%       y - Nx1 smoothed vector
%
%   Requires: base MATLAB only.
%
%   See also SMOOTHPATH.

if nargin < 3 || isempty(passes), passes = 1; end

requireInput(isnumeric(x) && isvector(x) && all(isfinite(x)), 'smoothSeries', 'x must be a finite vector');
requireInput(isscalar(halfWin) && halfWin >= 0 && halfWin == round(halfWin), ...
             'smoothSeries', 'halfWin must be a non-negative integer');

wasRow = isrow(x);
y = x(:);
N = numel(y);
if halfWin == 0 || N < 3
    if wasRow, y = y.'; end
    return;
end

lo = max((1:N).' - halfWin, 1);
hi = min((1:N).' + halfWin, N);
cnt = hi - lo + 1;
for p = 1:passes
    % Shrinking-window moving average via cumulative sums (Phase 2, speed).
    C = [0; cumsum(y)];
    y = (C(hi+1) - C(lo)) ./ cnt;
end

if wasRow, y = y.'; end
end
