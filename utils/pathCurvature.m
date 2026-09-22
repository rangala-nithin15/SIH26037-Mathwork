function k = pathCurvature(P)
%PATHCURVATURE Signed curvature at every point of a polyline.
%
%   COMPONENT STATUS: REAL
%
%   k = PATHCURVATURE(P) estimates the signed curvature (1/m) at each point
%   using the circumscribed-circle formula on consecutive point triples.
%   Positive curvature is a left turn, negative a right turn.
%
%   The triangle form is used rather than finite-difference derivatives
%   because it stays numerically stable on unevenly spaced points, which is
%   exactly what corridor extraction produces.
%
%       k = 2 * cross(v1, v2) / (|v1| * |v2| * |v2 - v1|)
%
%   Inputs:
%       P - Nx2 array of [x y] points, N >= 3
%
%   Outputs:
%       k - Nx1 vector of signed curvature in 1/m. Endpoints copy their
%           nearest interior neighbour. Degenerate (coincident) triples
%           yield 0 rather than Inf or NaN.
%
%   Example:
%       k = pathCurvature(circlePoints);   % ~ 1/R everywhere
%
%   Requires: base MATLAB only.
%
%   See also PATHHEADING, CHECKFEASIBILITY.

requireInput(isnumeric(P) && size(P,2) == 2 && all(isfinite(P(:))), 'pathCurvature', 'P must be a finite Nx2');
N = size(P,1);
k = zeros(N,1);
if N < 3
    return;
end

% Vectorised circumscribed-circle curvature (Phase 2, for speed).
v1 = P(2:N-1,:) - P(1:N-2,:);
v2 = P(3:N,:)   - P(2:N-1,:);
v3 = P(3:N,:)   - P(1:N-2,:);
n1 = sqrt(sum(v1.^2, 2));
n2 = sqrt(sum(v2.^2, 2));
n3 = sqrt(sum(v3.^2, 2));
denom  = n1 .* n2 .* n3;
crossZ = v1(:,1) .* v2(:,2) - v1(:,2) .* v2(:,1);
kin = 2 * crossZ ./ max(denom, 1e-300);
kin(denom < 1e-9) = 0;                     % coincident points: no curvature
k(2:N-1) = kin;

k(1) = k(2);
k(N) = k(N-1);
end
