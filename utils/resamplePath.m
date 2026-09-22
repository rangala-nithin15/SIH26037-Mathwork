function [Q, sq] = resamplePath(P, n)
%RESAMPLEPATH Resample a polyline to n points equally spaced in arc length.
%
%   COMPONENT STATUS: REAL
%
%   [Q, sq] = RESAMPLEPATH(P, n) returns n points along P that are evenly
%   spaced by distance travelled. Corridor boundaries extracted from sensor
%   data arrive unevenly spaced; almost every downstream step (smoothing,
%   curvature, deformation) assumes uniform spacing, so this is applied
%   first.
%
%   Inputs:
%       P - Mx2 array of [x y] points, M >= 1
%       n - positive integer, number of output points (n >= 2 recommended)
%
%   Outputs:
%       Q  - nx2 array of resampled points
%       sq - nx1 arc-length coordinate of each output point
%
%   Degenerate inputs are handled explicitly rather than being allowed to
%   produce NaN: a single input point, or a path of zero total length, is
%   replicated n times.
%
%   Example:
%       Q = resamplePath([0 0; 10 0], 5);   % 5 points spaced 2.5 m apart
%
%   Requires: base MATLAB only.
%
%   See also PATHARCLENGTH, SMOOTHPATH.

requireInput(isnumeric(P) && size(P,2) == 2 && ~isempty(P) && all(isfinite(P(:))), ...
             'resamplePath', 'P must be a non-empty finite Nx2');
requireInput(isscalar(n) && n >= 1 && n == round(n), 'resamplePath', 'n must be a positive integer');

if size(P,1) == 1
    Q  = repmat(P, n, 1);
    sq = zeros(n,1);
    return;
end

s = pathArcLength(P);
L = s(end);

if L <= eps
    % All points coincide: nothing to interpolate along.
    Q  = repmat(P(1,:), n, 1);
    sq = zeros(n,1);
    return;
end

% Duplicate points give repeated s values, which interp1 rejects. Keep only
% strictly increasing stations.
keep    = [true; diff(s) > 1e-9];
sUnique = s(keep);
Punique = P(keep,:);

if numel(sUnique) < 2
    Q  = repmat(P(1,:), n, 1);
    sq = zeros(n,1);
    return;
end

sq = linspace(0, sUnique(end), n).';
Q  = [interp1(sUnique, Punique(:,1), sq, 'linear'), ...
      interp1(sUnique, Punique(:,2), sq, 'linear')];
end
