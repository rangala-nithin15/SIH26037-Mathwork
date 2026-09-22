function mask = isFollowingRoadUser(ego, obstacles, vp)
%ISFOLLOWINGROADUSER Road users BEHIND the ego travelling the same way.
%
%   COMPONENT STATUS: REAL
%
%   mask = ISFOLLOWINGROADUSER(ego, obstacles, vp) is true for each road
%   user whose centre is behind the ego's rear bumper and whose heading is
%   within 60 degrees of the ego's, i.e. following traffic.
%
%   Why (Phase 2): a constant-velocity prediction of a faster vehicle behind
%   the ego says it will drive into the ego. The planner used to treat that
%   as a conflict ON ITS OWN TRAJECTORY: predicted risk 1.0, planned TTC
%   below critical, emergency braking -- which, with a vehicle close behind,
%   is exactly the wrong reaction and made the predicted conflict worse
%   every cycle. In the demo run this single effect caused most of the
%   emergency stops.
%
%   Following traffic is therefore excluded from the ego's BRAKING and
%   STOPPING decisions (conflict risk, planned TTC, clearance check, lead
%   following). It is NOT excluded from the lateral risk grid, so the ego
%   still does not swerve into a vehicle that is overtaking it.
%
%   Stated limitation: the ego does not reason about being rear-ended. A
%   follower that does not keep its distance is not something this planner
%   can prevent; the scenario's follower actors keep a gap (MAKEACTOR
%   'Follower').
%
%   Inputs:
%       ego       - ego state
%       obstacles - obstacle struct array
%       vp        - vehicle params
%
%   Outputs:
%       mask - 1xM logical
%
%   Requires: base MATLAB only.
%
%   See also IRPSCPLANNER, CONFLICTRISK, CHECKCLEARANCE.

M = numel(obstacles);
mask = false(1, M);
fwd = [cos(ego.heading), sin(ego.heading)];
for k = 1:M
    o = obstacles(k);
    along = (o.pos - ego.pos) * fwd.';
    sameDir = cos(o.heading - ego.heading) > 0.5 || o.speed < 0.5 && along < 0;
    mask(k) = along + o.length/2 < -vp.rearOverhang && sameDir;
end
end
