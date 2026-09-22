function [smoothness, details] = metricSmoothness(log)
%METRICSMOOTHNESS Smoothness of the path the vehicle actually drove.
%
%   COMPONENT STATUS: REAL
%
%   [smoothness, details] = METRICSMOOTHNESS(log) measures how smooth the
%   executed trajectory was. LOWER IS SMOOTHER.
%
%   The measure is the root mean square rate of change of curvature along
%   the driven path:
%
%       smoothness = sqrt( mean( (dk/ds)^2 ) )
%
%   Rate of change of curvature is used rather than curvature itself,
%   because curvature alone would penalise a smooth constant-radius bend
%   just as heavily as an erratic wobble. What matters for ride quality and
%   for steering effort is how fast the curvature CHANGES, and that is what
%   distinguishes a planner that commits to one clean avoidance manoeuvre
%   from one that hunts between options.
%
%   This is a headline metric for the baseline comparison: a fixed-candidate
%   planner switching between discrete lateral offsets produces visible
%   steps in curvature, and this metric is what makes that visible as a
%   number rather than only in a plot.
%
%   Inputs:
%       log - simulation log struct with field .egoPos (Nx2)
%
%   Outputs:
%       smoothness - RMS of dk/ds (1/m^2). NaN if the path is too short.
%       details    - struct with .meanAbsCurvature, .maxAbsCurvature,
%                    .pathLength
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, PATHCURVATURE.

details = struct('meanAbsCurvature', NaN, 'maxAbsCurvature', NaN, ...
                 'pathLength', 0);

if ~isfield(log, 'egoPos') || size(log.egoPos,1) < 4
    smoothness = NaN;
    return;
end

P = log.egoPos;

% Remove points where the vehicle was essentially stationary. Curvature is
% undefined there and including it produces enormous spurious values.
d    = [0; sqrt(sum(diff(P,1,1).^2, 2))];
keep = d > 1e-3;
keep(1) = true;
P = P(keep, :);

if size(P,1) < 4
    smoothness = NaN;
    return;
end

s = pathArcLength(P);
k = pathCurvature(P);

details.meanAbsCurvature = mean(abs(k));
details.maxAbsCurvature  = max(abs(k));
details.pathLength       = s(end);

ds = diff(s);
ds(ds < 1e-6) = 1e-6;
dk = diff(k);

smoothness = sqrt(mean((dk ./ ds).^2));
end
