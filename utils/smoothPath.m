function Q = smoothPath(P, halfWin, passes, lockEnds)
%SMOOTHPATH Moving-average smoothing of a polyline with edge handling.
%
%   COMPONENT STATUS: REAL
%
%   Q = SMOOTHPATH(P, halfWin, passes, lockEnds) smooths P with a centred
%   moving average of half-width halfWin, applied `passes` times. Repeated
%   short passes approximate Gaussian smoothing while staying cheap and
%   dependency-free.
%
%   Inputs:
%       P        - Nx2 array of [x y] points
%       halfWin  - non-negative integer half-window (0 = no smoothing)
%       passes   - positive integer, number of smoothing passes (default 1)
%       lockEnds - logical, hold the first and last point fixed (default true)
%
%   Outputs:
%       Q - Nx2 array of smoothed points
%
%   Why lockEnds defaults to true
%   -----------------------------
%   The first trajectory point must stay at the ego vehicle's actual
%   position, otherwise the controller is handed a path starting somewhere
%   the vehicle is not. Smoothing that drags the start point away would
%   inject a phantom lateral error.
%
%   Edges are handled by shrinking the window rather than zero-padding;
%   zero-padding would pull the ends toward the origin.
%
%   Example:
%       Q = smoothPath(noisyPath, 3, 2);
%
%   Requires: base MATLAB only.
%
%   See also RESAMPLEPATH, SMOOTHSERIES.

if nargin < 3 || isempty(passes),   passes   = 1;    end
if nargin < 4 || isempty(lockEnds), lockEnds = true; end

requireInput(isnumeric(P) && size(P,2) == 2 && ~isempty(P) && all(isfinite(P(:))), ...
             'smoothPath', 'P must be a non-empty finite Nx2');
requireInput(isscalar(halfWin) && halfWin >= 0 && halfWin == round(halfWin), ...
             'smoothPath', 'halfWin must be a non-negative integer');
requireInput(isscalar(passes) && passes >= 1 && passes == round(passes), ...
             'smoothPath', 'passes must be a positive integer');

Q = P;
N = size(P,1);
if halfWin == 0 || N < 3
    return;
end

first = P(1,:);
last  = P(end,:);

lo = max((1:N).' - halfWin, 1);
hi = min((1:N).' + halfWin, N);
cnt = hi - lo + 1;
for p = 1:passes
    % Shrinking-window moving average via cumulative sums (Phase 2, for
    % speed; identical to averaging Q(lo:hi,:) at each point).
    C = [zeros(1,2); cumsum(Q, 1)];
    Q = (C(hi+1,:) - C(lo,:)) ./ [cnt cnt];
end

if lockEnds
    Q(1,:)   = first;
    Q(end,:) = last;
end
end
