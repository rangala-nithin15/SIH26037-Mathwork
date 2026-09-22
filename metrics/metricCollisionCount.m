function n = metricCollisionCount(log)
%METRICCOLLISIONCOUNT Number of distinct collision events in a run.
%
%   COMPONENT STATUS: REAL
%
%   n = METRICCOLLISIONCOUNT(log) counts distinct collision EVENTS, not
%   collision time steps. A single collision lasting twelve steps is one
%   event, counted once. Counting steps instead would make a slow scrape
%   look twelve times worse than a fast impact, which is backwards.
%
%   An event is a rising edge in log.collided.
%
%   IMPORTANT: a count of zero means no collision occurred in THIS
%   simulation run, under a simplified sensor model and a kinematic vehicle
%   model. It is not evidence of collision avoidance in general and must
%   never be reported as such.
%
%   Inputs:
%       log - simulation log struct with field .collided (Nx1 logical)
%
%   Outputs:
%       n - non-negative integer count of collision events
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, METRICCLEARANCE.

if ~isfield(log, 'collided') || isempty(log.collided)
    n = 0;
    return;
end

c = logical(log.collided(:));
n = sum(c & [true; ~c(1:end-1)]);
end
