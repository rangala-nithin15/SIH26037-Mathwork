function out = plannerOutputTemplate()
%PLANNEROUTPUTTEMPLATE Default planner output with every field present.
%
%   COMPONENT STATUS: REAL
%
%   out = PLANNEROUTPUTTEMPLATE() returns the output struct shared by
%   IRPSCPLANNER and BASELINEPLANNER, with safe default values. Both
%   planners start from this template, so the simulation loop, the metrics
%   and the viewer can rely on every field existing on every return path,
%   whichever planner produced it.
%
%   See IRPSCPLANNER for the meaning of each field.
%
%   Requires: base MATLAB only.
%
%   See also IRPSCPLANNER, BASELINEPLANNER.

out = struct();
out.isSafeStop          = false;
out.status              = 'ok';
out.traj                = [];
out.rejectedTraj        = [];       % candidate rejected by clearance/feasibility, if any
out.corridor            = [];
out.preds               = [];
out.confidence          = 1;
out.confidenceBreakdown = struct();
out.risk                = 0;
out.ttc                 = Inf;
out.ttcLevel            = 'none';
out.ttcPlanned          = Inf;
out.emergencyBrake      = false;
out.requiredDecel       = 0;
out.feasible            = true;
out.clearanceOk         = true;
out.minClearance        = Inf;
out.feasibilityDetails  = struct();
out.clearanceDetails    = struct();
out.score               = Inf;
out.behaviour           = struct('avoiding', false, 'slowing', false, ...
                                 'following', false, 'yielding', false, ...
                                 'narrowPassage', false, 'blockedAhead', false, ...
                                 'potholeAvoid', false, 'potholeSlow', false, ...
                                 'lead', struct('active', false, 'id', NaN, ...
                                 'class', '', 'gap', Inf, 'leadSpeed', NaN), ...
                                 'reason', 'cruise along drivable corridor');
out.potholes            = [];
out.preferredPath       = zeros(0,2);
out.riskGrid            = struct('R', [], 'offsets', [], 'potholeCost', [], ...
                                 's', [], 'center', zeros(0,2), 'heading', []);
out.state               = struct('center', [], 'profile', [], 'trajPos', []);
out.timing              = struct();
end
