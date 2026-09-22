function scn = scenarioCattleCrossing(res)
%SCENARIOCATTLECROSSING SIH scenario E: sudden cattle crossing.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION
%
%   scn = SCENARIOCATTLECROSSING(res) builds an otherwise clear road on
%   which cattle step out in front of the vehicle at a scripted moment.
%
%   WHAT THIS SCENARIO IS DESIGNED TO TEST
%   --------------------------------------
%   The emergency response path, end to end: detection of a sudden hazard,
%   time-to-conflict escalation, the decision logic's emergency override
%   that bypasses debounce, and the safe-stop trajectory.
%
%   It is the simplest scenario geometrically and the most demanding
%   temporally. Everything is clear, the vehicle is at speed and settled in
%   NORMAL_DRIVING, and then a large slow hazard appears with very little
%   warning.
%
%   Design decisions that make this a fair test:
%     - The cattle are activated at t = 6 s, by which point the vehicle has
%       been driving normally long enough for the decision logic to have
%       genuinely settled. Triggering at t = 0 would test nothing.
%     - Two cattle at different speeds, so the hazard is a moving group with
%       a gap rather than a single object. A planner that aims for the gap
%       must account for the gap closing.
%     - Cattle are given a high lateral-spread value in the prediction
%       config, correctly reflecting that an animal's next move is far less
%       predictable than a vehicle's.
%     - A motorcycle follows close behind the ego. This makes braking
%       CONSEQUENTIAL rather than free: an unnecessarily hard stop is not a
%       cost-free choice. It is included so the trade-off is visible in the
%       metrics, not because the current planner reasons about followers --
%       it does not, and that is stated openly as a limitation.
%     - The road is wide enough that steering around IS possible, so the
%       planner has a genuine choice between braking and avoiding rather
%       than a forced single answer.
%
%   Inputs:
%       res - grid resolution in metres per cell
%
%   Outputs:
%       scn - scenario struct (see buildScenario for the field list)
%
%   Requires: base MATLAB only.
%
%   See also BUILDSCENARIO, SAFESTOPTRAJECTORY, DECISIONLOGIC, TIMETOCONFLICT.

% --- Open rural road, wide enough to allow avoidance --------------------
sVals = (0:2:160).';
xC = sVals;
yC = 4*sin(sVals/55);
centerline = [xC, yC];

halfWidth = 3.9 + 0.25*sin(sVals/21);

extras = struct('center', {}, 'radius', {}, 'halfSize', {}, 'yaw', {});
% Sparse roadside vegetation, which partially occludes the approach.
for xb = 30:25:150
    j = find(sVals >= xb, 1);
    p = centerline(j,:) + (halfWidth(j) + 0.8) * [0, 1];
    extras(end+1) = struct('center', p, 'radius', 1.2, ...
                           'halfSize', [], 'yaw', []); %#ok<AGROW>
end

grid = buildRoadGrid(centerline, halfWidth, res, extras);

% --- Actors --------------------------------------------------------------
% The trigger: cattle stepping out at t = 6 s, ahead of the ego.
jCross = find(sVals >= 92, 1);
crossPt = centerline(jCross,:);

actors = makeActor(1, 'animal', ...
                   [crossPt + [0, -7.5]; crossPt + [1.0, 7.5]], ...
                   1.1, 'StartTime', 6.0);

% A second cow, slower and slightly behind: the gap between them closes.
actors(end+1) = makeActor(2, 'animal', ...
                          [crossPt + [-3.5, -8.0]; crossPt + [-2.5, 7.0]], ...
                          0.75, 'StartTime', 6.8);

% Motorcycle following the ego closely: braking is not cost-free.
% Phase 2: road-following path, and a Follower -- it keeps a gap instead
% of driving through the ego from behind (which the referee counted as the
% planner's collision before the cattle had even appeared).
% Starts 1.5 s after the ego so it is BEHIND it: at t = 0 the two bodies
% overlapped, which the referee recorded as a contact at t = 0.2 s.
actors(end+1) = makeActor(3, 'motorcycle', roadPath(centerline, 0, 160, 0.9), ...
                          13.5, 'StartTime', 1.5, 'Follower', true);

% Oncoming truck, so swerving right into the opposing side is not free either.
actors(end+1) = makeActor(4, 'truck', roadPath(centerline, 160, 0, -2.0), ...
                          10.0, 'StartTime', 0);

% --- Assemble ------------------------------------------------------------
scn.name        = 'cattle_crossing';
scn.profile     = 'cattle';
scn.grid        = grid;
scn.centerline  = centerline;
scn.egoStart    = struct('pos', centerline(2,:), 'heading', 0, 'speed', 11.0);
scn.goal        = centerline(end-2,:);
scn.goalRadius  = 6.0;
scn.actors      = actors;
scn.duration    = 35.0;
scn.roadHalfWidth = halfWidth;
scn.extras      = extras;
scn.extraTypes  = repmat({'bush'}, 1, numel(extras));
scn.sihScenario = 'E: Sudden cattle crossing';
scn.description = ['Open rural road. Two cattle step into the road at ' ...
                   't = 6.0 s and t = 6.8 s at different speeds, so the gap ' ...
                   'between them closes. A motorcycle follows the ego ' ...
                   'closely and an oncoming truck occupies the opposing ' ...
                   'side, so neither braking hard nor swerving is free. ' ...
                   'Tests the emergency escalation and safe-stop path.'];
end
