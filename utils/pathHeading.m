function th = pathHeading(P)
%PATHHEADING Tangent heading at every point of a polyline.
%
%   COMPONENT STATUS: REAL
%
%   th = PATHHEADING(P) returns the heading (radians, atan2 convention) of
%   the path tangent at each point. Central differences are used in the
%   interior and one-sided differences at the ends.
%
%   Inputs:
%       P - Nx2 array of [x y] points, N >= 2
%
%   Outputs:
%       th - Nx1 vector of headings in radians, wrapped to (-pi, pi]
%
%   Example:
%       th = pathHeading([0 0; 1 0; 2 1]);
%
%   Requires: base MATLAB only.
%
%   See also PATHCURVATURE, WRAPToPi.

requireInput(isnumeric(P) && size(P,2) == 2 && all(isfinite(P(:))), 'pathHeading', 'P must be a finite Nx2');
N = size(P,1);
if N < 2
    error('pathHeading:tooShort', 'Need at least 2 points, got %d.', N);
end

dx = zeros(N,1);
dy = zeros(N,1);

dx(1) = P(2,1) - P(1,1);
dy(1) = P(2,2) - P(1,2);
dx(N) = P(N,1) - P(N-1,1);
dy(N) = P(N,2) - P(N-1,2);

if N > 2
    dx(2:N-1) = P(3:N,1) - P(1:N-2,1);
    dy(2:N-1) = P(3:N,2) - P(1:N-2,2);
end

th = atan2(dy, dx);
end
