function [n, details] = metricEmergencyStops(log)
%METRICEMERGENCYSTOPS Number of times the system escalated to SAFE_STOP.
%
%   COMPONENT STATUS: REAL
%
%   [n, details] = METRICEMERGENCYSTOPS(log) counts distinct entries into
%   the SAFE_STOP state, together with how long the vehicle spent there.
%
%   How to interpret this metric -- it is NOT simply "lower is better"
%   ------------------------------------------------------------------
%   A safe stop is the system working as designed when no feasible path
%   exists. Zero stops in a scenario containing a genuine blockage would be
%   the alarming result, not the good one.
%
%   The metric is diagnostic, and it is most informative in comparison:
%   if IR-PSC and the baseline face identical scenarios and the baseline
%   stops far more often, that indicates the baseline could not find paths
%   IR-PSC could. If IR-PSC stops more often, that is a finding to
%   investigate and report, not to hide.
%
%   Consecutive frames in SAFE_STOP are one event, counted once.
%
%   Inputs:
%       log - simulation log struct with field .state (Nx1 cellstr)
%
%   Outputs:
%       n       - number of distinct SAFE_STOP episodes
%       details - struct with .totalFrames, .longestEpisodeFrames,
%                 .fractionOfRun
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, DECISIONLOGIC.

details = struct('totalFrames', 0, 'longestEpisodeFrames', 0, ...
                 'fractionOfRun', 0);
n = 0;

if ~isfield(log,'state') || isempty(log.state)
    return;
end

isStop = false(numel(log.state),1);
for i = 1:numel(log.state)
    isStop(i) = strcmp(log.state{i}, 'SAFE_STOP');
end

n = sum(isStop & [true; ~isStop(1:end-1)]);

details.totalFrames   = sum(isStop);
details.fractionOfRun = details.totalFrames / max(numel(isStop), 1);

% Longest consecutive run of SAFE_STOP frames.
best = 0; run = 0;
for i = 1:numel(isStop)
    if isStop(i)
        run = run + 1;
        best = max(best, run);
    else
        run = 0;
    end
end
details.longestEpisodeFrames = best;
end
