function s = pathArcLength(P)
%PATHARCLENGTH Cumulative arc length along a polyline.
%
%   COMPONENT STATUS: REAL
%
%   s = PATHARCLENGTH(P) returns the cumulative distance travelled along the
%   polyline P, starting at 0 for the first point.
%
%   Inputs:
%       P - Nx2 array of [x y] points, N >= 1
%
%   Outputs:
%       s - Nx1 vector of cumulative arc lengths, s(1) = 0, non-decreasing
%
%   Example:
%       s = pathArcLength([0 0; 3 4; 3 8]);   % -> [0; 5; 9]
%
%   Requires: base MATLAB only.
%
%   See also RESAMPLEPATH, PATHCURVATURE.

requireInput(isnumeric(P) && size(P,2) == 2 && ~isempty(P) && all(isfinite(P(:))), ...
             'pathArcLength', 'P must be a non-empty finite Nx2');

if size(P,1) == 1
    s = 0;
    return;
end

d = sqrt(sum(diff(P,1,1).^2, 2));   % segment lengths, (N-1)x1
s = [0; cumsum(d)];
end
