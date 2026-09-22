function [meanLat, maxLat, p95Lat] = metricReplanLatency(log)
%METRICREPLANLATENCY Wall-clock time taken by the planner per replan.
%
%   COMPONENT STATUS: REAL (measurement) / see the caveat below
%
%   [meanLat, maxLat, p95Lat] = METRICREPLANLATENCY(log) summarises how long
%   each planning cycle took.
%
%   The 95th percentile is reported alongside the mean because real-time
%   behaviour is governed by the tail, not the average. A planner averaging
%   20 ms but occasionally taking 300 ms will miss its deadline, and the
%   mean alone hides that entirely.
%
%   IMPORTANT CAVEAT ON INTERPRETATION
%   ----------------------------------
%   These are MATLAB interpreted-execution timings on a development
%   machine, measured with tic/toc. They are affected by other processes,
%   MATLAB's JIT warm-up and the machine's power state.
%
%   They are meaningful for RELATIVE comparison -- IR-PSC versus baseline on
%   the same machine in the same session -- which is what this project uses
%   them for. They must NOT be quoted as embedded real-time performance.
%   Establishing that would require generated code on target hardware, which
%   this project has not done.
%
%   Inputs:
%       log - simulation log struct with field .planTime (Nx1 seconds, NaN
%             on steps where no replan occurred)
%
%   Outputs:
%       meanLat - mean replan time (s), NaN if never measured
%       maxLat  - worst replan time (s), NaN if never measured
%       p95Lat  - 95th percentile replan time (s), NaN if never measured
%
%   Requires: base MATLAB only (percentile computed by sorting, so no
%   Statistics Toolbox dependency).
%
%   See also COMPUTEMETRICS, IRPSCPLANNER.

if ~isfield(log, 'planTime') || isempty(log.planTime)
    meanLat = NaN;  maxLat = NaN;  p95Lat = NaN;
    return;
end

t = log.planTime(:);
t = t(isfinite(t) & t >= 0);

if isempty(t)
    meanLat = NaN;  maxLat = NaN;  p95Lat = NaN;
    return;
end

meanLat = mean(t);
maxLat  = max(t);

ts = sort(t);
idx = max(1, min(numel(ts), ceil(0.95 * numel(ts))));
p95Lat = ts(idx);
end
