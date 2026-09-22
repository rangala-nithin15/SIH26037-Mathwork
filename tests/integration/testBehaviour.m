function tests = testBehaviour
%TESTBEHAVIOUR Closed-loop BEHAVIOURAL tests (Phase 2).
%
%   COMPONENT STATUS: REAL
%
%   The original integration tests only checked that a run finished without
%   an exception; a vehicle that never moved passed all of them. These tests
%   check what the vehicle actually DOES, on small synthetic roads that run
%   in seconds:
%
%     - it moves, and reaches the goal on a clear road
%     - it pulls away from standstill (no permanent stall)
%     - it avoids a static obstacle without touching it
%     - it stops, in a controlled way, before a road that is fully blocked
%     - a pothole is detected, confirmed and its severity estimated
%     - a severe pothole is avoided when there is room
%     - a pothole that cannot be avoided is crossed slowly
%     - it follows a slow road user it cannot pass, without contact
%     - it does not hit a pedestrian crossing in front of it
%     - it replans: the trajectory changes when a hazard appears
%     - the collision referee detects driving into a static obstacle
%
%   Run: runtests('testBehaviour')   (MATLAB)
%        runOctaveTests({'testBehaviour'}) after setupOctave (GNU Octave)
%
%   Requires: base MATLAB only.

tests = functiontests(localfunctions);
end

% =========================================================================
function scn = straightRoad(len, halfWidth, extras)
if nargin < 3, extras = []; end
res = 0.2;
cl  = [(0:2:len).', zeros(numel(0:2:len), 1)];
scn.name        = 'test_straight';
scn.profile     = 'village';
if isempty(extras)
    extras = struct('center', {}, 'radius', {}, 'halfSize', {}, 'yaw', {});
end
scn.grid        = buildRoadGrid(cl, halfWidth, res, extras);
scn.centerline  = cl;
scn.roadHalfWidth = repmat(halfWidth, size(cl,1), 1);
scn.egoStart    = struct('pos', [4 0], 'heading', 0, 'speed', 6.0);
scn.goal        = [len - 14, 0];      % the road is a dead end: the car stops ~10 m before it
scn.goalRadius  = 4.0;
scn.actors      = makeActor(99, 'car', [len+500 50; len+600 50], 0, 'StartTime', 1e6);
scn.duration    = 40;
scn.sihScenario = 'test';
scn.description = 'synthetic test road';
end

function [log, M] = run(scn, opts)
base = struct('seed', 3, 'maxTime', scn.duration);
f = fieldnames(opts);
for i = 1:numel(f), base.(f{i}) = opts.(f{i}); end
[log, M] = runScenario(scn, @irpscPlanner, [], base);
end

% =========================================================================
function testVehicleMovesAndReachesGoalOnClearRoad(tc)
scn = straightRoad(120, 3.2);
[log, M] = run(scn, struct('usePerfectPerception', true));
verifyTrue(tc, log.goalReached, 'goal not reached on an empty straight road');
verifyEqual(tc, M.collisionCount, 0);
verifyGreaterThan(tc, M.averageSpeed, 6.0);
verifyEqual(tc, M.emergencyStops, 0, 'safe stop on an empty road');
end

function testPullsAwayFromStandstill(tc)
scn = straightRoad(120, 3.2);
scn.egoStart.speed = 0;
[log, M] = run(scn, struct('usePerfectPerception', true, 'maxTime', 10));
travelled = log.egoPos(end,1) - log.egoPos(1,1);
verifyGreaterThan(tc, travelled, 30, 'vehicle did not pull away from rest');
verifyFalse(tc, M.stopDetails.endsStopped);
end

function testAvoidsStaticObstacleWithoutContact(tc)
% A parked car occupies the left 1.8 m of a 7 m road at x = 50.
ex = struct('center', [50 2.3], 'radius', [], 'halfSize', [2.1 0.9], 'yaw', 0);
scn = straightRoad(120, 3.5, ex);
[log, M] = run(scn, struct('usePerfectPerception', true));
verifyTrue(tc, log.goalReached);
verifyEqual(tc, sum(log.collidedStatic), 0, 'drove into the parked car');
near = abs(log.egoPos(:,1) - 50) < 3;
verifyLessThan(tc, max(log.egoPos(near,2)), 0.0, 'did not move away from the obstacle');
verifyGreaterThanOrEqual(tc, min(log.clearStatic), 0.25 - 0.2);
end

function testStopsBeforeFullyBlockedRoad(tc)
ex = struct('center', [60 0], 'radius', [], 'halfSize', [1.0 4.0], 'yaw', 0);
scn = straightRoad(120, 3.2, ex);
[log, M] = run(scn, struct('usePerfectPerception', true, 'maxTime', 20));
verifyFalse(tc, log.goalReached);
verifyEqual(tc, M.collisionCount, 0, 'contact with the blockage');
verifyLessThan(tc, log.egoSpeed(end), 0.1, 'not stopped');
vp = vehicleParams(irpscConfig('village'));
frontBumper = log.egoPos(end,1) + vp.frontOverhang;
verifyLessThan(tc, frontBumper, 59.0);
verifyEqual(tc, sum(log.emergencyBrake), 0, 'a blockage seen from afar needs no emergency braking');
end

function testPotholeDetectedConfirmedAndClassified(tc)
scn = straightRoad(120, 3.5);
scn.potholes = makePothole(1, [45 -1.2], 1.2, 1.0, 0.12);    % severe, off-centre
[log, M] = run(scn, struct('usePerfectPerception', false, 'maxTime', 12));
verifyEqual(tc, M.pothole.nConfirmed, 1, 'pothole not confirmed by the tracker');
tr = log.potholeTracks{end};
k = find([tr.confirmed], 1);
verifyNotEmpty(tc, k);
verifyEqual(tc, tr(k).truthId, 1);
verifyLessThan(tc, hypot(tr(k).pos(1) - 45, tr(k).pos(2) + 1.2), 0.5, 'position estimate');
verifyEqual(tc, tr(k).severity, 'severe');
end

function testSeverePotholeAvoidedWhenThereIsRoom(tc)
scn = straightRoad(120, 3.5);
scn.potholes = makePothole(1, [50 0.3], 1.4, 1.6, 0.14);     % severe, in the path
[log, M] = run(scn, struct('usePerfectPerception', true));
verifyTrue(tc, log.goalReached);
verifyEqual(tc, M.pothole.wheelEntries, 0, 'drove into an avoidable severe pothole');
end

function testUnavoidablePotholeCrossedSlowly(tc)
% A 2.9 m wide road with a moderate pothole across its whole usable width.
scn = straightRoad(90, 1.45);          % 3 m/s narrow-passage cap: keep it short
scn.potholes = makePothole(1, [55 0], 1.0, 2.9, 0.07);       % moderate
[log, M] = run(scn, struct('usePerfectPerception', true));
verifyTrue(tc, log.goalReached, 'did not continue after the pothole');
verifyGreaterThan(tc, M.pothole.wheelEntries, 0);
cfg = irpscConfig('village');
verifyLessThanOrEqual(tc, M.pothole.maxEntrySpeed, cfg.pothole.speed.moderate + 0.75);
end

function testFollowsSlowBicycleWithoutContact(tc)
% Single-lane road: no room to pass the bicycle.
scn = straightRoad(160, 1.6);
scn.actors = makeActor(1, 'bicycle', [20 0; 200 0], 3.0, 'StartTime', 0);
[log, M] = run(scn, struct('usePerfectPerception', true, 'maxTime', 25));
verifyEqual(tc, M.collisionCount, 0, 'hit the bicycle');
late = log.t > 15;
verifyLessThan(tc, mean(log.egoSpeed(late)), 4.5, 'did not slow to the bicycle''s speed');
verifyGreaterThan(tc, mean(log.egoSpeed(late)), 1.5, 'stopped instead of following');
verifyEqual(tc, sum(log.emergencyBrake), 0);
end

function testNoContactWithCrossingPedestrian(tc)
scn = straightRoad(120, 3.5);
scn.actors = makeActor(1, 'pedestrian', [45 -5; 45 5], 1.3, 'StartTime', 2.0);
[log, M] = run(scn, struct('usePerfectPerception', false));
verifyEqual(tc, M.collisionCount, 0, 'contact with a crossing pedestrian');
verifyTrue(tc, log.goalReached);
end

function testTrajectoryIsReplannedWhenHazardAppears(tc)
ex = struct('center', [45 2.0], 'radius', [], 'halfSize', [2.0 1.0], 'yaw', 0);
scn = straightRoad(100, 3.5, ex);
[log, ~] = run(scn, struct('usePerfectPerception', true, 'maxTime', 8));
k = find(log.replanned);
verifyGreaterThan(tc, numel(k), 10);
lat = zeros(numel(k),1);
for i = 1:numel(k)
    tr = log.traj{k(i)};
    lat(i) = min(tr.pos(:,2));
end
verifyGreaterThan(tc, max(lat) - min(lat), 0.4, 'trajectory never changed shape');
end

function testRefereeDetectsDrivingIntoStaticObstacle(tc)
% A deliberately naive planner that ignores the grid: the referee must
% record the contact. This is the check that static obstacles cannot be
% driven through unnoticed.
ex = struct('center', [30 0], 'radius', [], 'halfSize', [1.0 4.0], 'yaw', 0);
scn = straightRoad(80, 3.2, ex);
naive = @(grid, ego, obs, cfg, st, hz) naivePlanner(ego, cfg);
[log, M] = runScenario(scn, naive, [], struct('seed', 1, 'maxTime', 6, ...
                                            'usePerfectPerception', true));
verifyGreaterThan(tc, sum(log.collidedStatic), 0);
verifyGreaterThan(tc, M.collisionCount, 0);
end

function out = naivePlanner(ego, cfg)
out = plannerOutputTemplate();
s = (0:1:40).';
P = [ego.pos(1) + s, repmat(ego.pos(2), numel(s), 1)];
out.traj = struct('pos', P, 'heading', zeros(numel(s),1), 'curvature', zeros(numel(s),1), ...
                  'speed', repmat(8, numel(s), 1), 's', s, 'times', s / 8, ...
                  'reachable', true(numel(s),1), 'stopS', Inf, 'valid', true, ...
                  'isSafeStop', false, 'emergency', false);
out.corridor = struct('valid', true, 'center', P, 'length', 40);
out.state = struct();
end
