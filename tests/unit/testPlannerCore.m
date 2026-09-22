function tests = testPlannerCore
%TESTPLANNERCORE Unit tests for deformation, safety checks and the planner.
%
%   COMPONENT STATUS: REAL
%
%   Run with:  results = runtests('testPlannerCore')
%
%   NOT YET EXECUTED. MATLAB is not installed on the machine where this was
%   written. Run on the destination laptop and report the real result.
%
%   Requires: base MATLAB only.

tests = functiontests(localfunctions);
end

% =====================================================================
function [g, cfg, ego] = straightSetup()
g   = buildRoadGrid([0 0; 60 0], 3.0, 0.2);
cfg = irpscConfig('urban');
ego = makeEgoState([5 0], 0, 6.0);
end

% =====================================================================
function testDeformationStaysCentredWithNoHazard(tc)
% With zero risk everywhere, the deviation cost must keep the path centred.
N = 20;  offsets = -2:0.25:2;
R = zeros(N, numel(offsets));
cfg = irpscConfig();

[d, info] = deformTrajectory(R, offsets, cfg, 0, []);
verifyTrue(tc, info.feasible);
verifyLessThan(tc, max(abs(d)), 0.3);
end

function testDeformationAvoidsHazard(tc)
% Put a hazard on the centre offset at the middle stations. The chosen
% profile must move away from it there.
N = 20;  offsets = -2:0.25:2;
R = zeros(N, numel(offsets));
[~, jCentre] = min(abs(offsets));
R(8:12, jCentre-2:jCentre+2) = 0.9;
cfg = irpscConfig();

[d, info] = deformTrajectory(R, offsets, cfg, 0, []);
verifyTrue(tc, info.feasible);
verifyGreaterThan(tc, max(abs(d(8:12))), 0.4);
end

function testDeformationRespectsForbiddenCells(tc)
% Cells marked Inf must never be selected, whatever the risk elsewhere.
N = 15;  offsets = -2:0.25:2;
R = zeros(N, numel(offsets));
R(:, offsets > 0) = Inf;               % left side entirely forbidden
cfg = irpscConfig();

[d, info] = deformTrajectory(R, offsets, cfg, 0, []);
verifyTrue(tc, info.feasible);
verifyLessThanOrEqual(tc, max(d), 0);
end

function testDeformationInfeasibleWhenRowFullyBlocked(tc)
N = 10;  offsets = -2:0.25:2;
R = zeros(N, numel(offsets));
R(5, :) = Inf;                          % one station has no way through
cfg = irpscConfig();

[~, info] = deformTrajectory(R, offsets, cfg, 0, []);
verifyFalse(tc, info.feasible);
verifyEqual(tc, info.blockedRows, 5);
end

function testDeformationAblationReturnsZeroProfile(tc)
N = 12;  offsets = -2:0.25:2;
R = zeros(N, numel(offsets));
cfg = irpscConfig();
cfg.ablation.useDeformation = false;

d = deformTrajectory(R, offsets, cfg, 0.5, []);
verifyEqual(tc, d, zeros(N,1), 'AbsTol', 1e-12);
end

function testDeformationIsSmooth(tc)
% The smoothness term must prevent the profile jumping between extremes on
% adjacent stations.
N = 30;  offsets = -2:0.25:2;
% Phase 2: deterministic pseudo-random risk. The original used rand without
% a seed, so the test could pass or fail from run to run.
R = mod((1:N).' * (1:numel(offsets)) * 7919, 97) / 97 * 0.3;
cfg = irpscConfig();

d = deformTrajectory(R, offsets, cfg, 0, []);
verifyLessThan(tc, max(abs(diff(d))), 0.8);
end

function testSpeedProfileSlowsForCurvature(tc)
cfg = irpscConfig();
s = (0:1:20).';
kStraight = zeros(21,1);
kCurved   = repmat(0.15, 21, 1);

vStraight = speedProfile(s, kStraight, 15, 15, cfg, []);
vCurved   = speedProfile(s, kCurved,   15, 15, cfg, []);

verifyLessThan(tc, mean(vCurved), mean(vStraight));
end

function testSpeedProfileRespectsLateralAccelLimit(tc)
% Phase 2: the profile now starts at the vehicle's ACTUAL speed (the old
% profile silently set the first sample to the curve limit, i.e. assumed
% the car could shed 16 m/s instantly). Starting below the curve limit, the
% profile must never exceed the lateral-acceleration limit.
cfg = irpscConfig();
s = (0:1:30).';
k = repmat(0.2, 31, 1);
v = speedProfile(s, k, 20, 3.0, cfg, []);
latAcc = v.^2 .* abs(k);
verifyLessThanOrEqual(tc, max(latAcc), cfg.ego.maxLatAccel + 1e-6);
end

function testSpeedProfileStartsAtCurrentSpeedAndBrakesLegally(tc)
% Phase 2: entering the same bend too fast, the profile starts at the
% current speed and sheds it no faster than the braking limit allows.
cfg = irpscConfig();
s = (0:1:30).';
k = repmat(0.2, 31, 1);
v = speedProfile(s, k, 20, 12.0, cfg, []);
verifyEqual(tc, v(1), 12.0, 'AbsTol', 1e-9);
dv2 = -diff(v.^2) ./ (2 * diff(s));          % deceleration per segment
verifyLessThanOrEqual(tc, max(dv2), cfg.ego.maxDecel + 1e-6);
verifyLessThan(tc, v(end), 4.0);
end

function testSpeedProfileRespectsJerkOnStraightRoad(tc)
% Phase 2 regression: the old profile produced 17.7 m/s^3 jerk here and
% every plan was rejected by checkFeasibility.
cfg = irpscConfig('village');
vp  = vehicleParams(cfg);
s = linspace(0, 40, 41).';
for v0 = [0 6 11]
    v = speedProfile(s, zeros(41,1), cfg.ego.maxSpeed, v0, cfg, []);
    t = zeros(41,1);
    for i = 2:41
        t(i) = t(i-1) + (s(i)-s(i-1)) / max(0.5*(v(i)+v(i-1)), 0.1);
    end
    traj = struct('pos', [s zeros(41,1)], 'heading', zeros(41,1), ...
                  'curvature', zeros(41,1), 'speed', v, 's', s, 'times', t, ...
                  'valid', true);
    [ok, d] = checkFeasibility(traj, cfg, vp);
    verifyTrue(tc, ok, sprintf('v0=%g: %s', v0, strjoin(d.violations, ',')));
    verifyEmpty(tc, d.comfortViolations, sprintf('v0=%g: jerk %.2f', v0, d.maxJerk));
    verifyLessThanOrEqual(tc, d.maxJerk, cfg.ego.maxJerk + 1e-2);
end
end

function testSpeedProfileSlowsForRisk(tc)
cfg = irpscConfig();
s = (0:1:20).';
k = zeros(21,1);
vNoRisk = speedProfile(s, k, 12, 12, cfg, zeros(21,1));
vRisk   = speedProfile(s, k, 12, 12, cfg, repmat(0.8, 21, 1));
verifyLessThan(tc, mean(vRisk), mean(vNoRisk));
end

function testGenerateTrajectoryAnchoredAtEgo(tc)
% The first trajectory point must be exactly where the vehicle is, or the
% controller is handed a phantom lateral error.
[g, cfg, ego] = straightSetup();
corr = extractCorridor(g, ego, cfg, []);
d = zeros(size(corr.center,1),1);
traj = generateTrajectory(corr, d, ego, cfg, []);
verifyEqual(tc, traj.pos(1,:), ego.pos, 'AbsTol', 1e-9);
end

function testGenerateTrajectoryPointCount(tc)
[g, cfg, ego] = straightSetup();
corr = extractCorridor(g, ego, cfg, []);
d = zeros(size(corr.center,1),1);
traj = generateTrajectory(corr, d, ego, cfg, []);
verifySize(tc, traj.pos, [cfg.traj.numPoints 2]);
end

function testFeasibilityAcceptsGentleTrajectory(tc)
[g, cfg, ego] = straightSetup();
vp   = vehicleParams(cfg);
corr = extractCorridor(g, ego, cfg, []);
d    = zeros(size(corr.center,1),1);
traj = generateTrajectory(corr, d, ego, cfg, []);

[ok, details] = checkFeasibility(traj, cfg, vp);
verifyTrue(tc, ok, sprintf('violations: %s', strjoin(details.violations, ', ')));
end

function testFeasibilityRejectsImpossibleCurvature(tc)
cfg = irpscConfig();
vp  = vehicleParams(cfg);

traj.pos       = [(0:0.5:5).', zeros(11,1)];
traj.heading   = zeros(11,1);
traj.curvature = repmat(5.0, 11, 1);          % 0.2 m turning radius
traj.speed     = repmat(5.0, 11, 1);
traj.s         = (0:0.5:5).';
traj.times     = (0:0.1:1).';
traj.valid     = true;

[ok, details] = checkFeasibility(traj, cfg, vp);
verifyFalse(tc, ok);
verifyTrue(tc, ismember('curvature', details.violations));
end

function testConfidenceInUnitInterval(tc)
[g, cfg, ego] = straightSetup();
corr = extractCorridor(g, ego, cfg, []);
o    = makeObstacle(1, 'car', [30 0], [-5 0], 'Confidence', 0.8, 'Age', 10);
p    = predictObstacles(o, cfg, corr);

c = computeConfidence(corr, p, o, cfg);
verifyGreaterThanOrEqual(tc, c, 0);
verifyLessThanOrEqual(tc, c, 1);
end

function testConfidenceDropsWithPoorPerception(tc)
[g, cfg, ego] = straightSetup();
corr = extractCorridor(g, ego, cfg, []);

good = makeObstacle(1, 'car', [30 0], [-5 0], 'Confidence', 0.95, 'Age', 40);
poor = makeObstacle(1, 'car', [30 0], [-5 0], 'Confidence', 0.25, 'Age', 1);

pGood = predictObstacles(good, cfg, corr);
pPoor = predictObstacles(poor, cfg, corr);

cGood = computeConfidence(corr, pGood, good, cfg);
cPoor = computeConfidence(corr, pPoor, poor, cfg);

verifyGreaterThan(tc, cGood, cPoor);
end

function testConfidenceAblationForcesOne(tc)
[g, cfg, ego] = straightSetup();
cfg.ablation.useConfidenceAware = false;
corr = extractCorridor(g, ego, cfg, []);
poor = makeObstacle(1, 'car', [30 0], [-5 0], 'Confidence', 0.1, 'Age', 1);
p    = predictObstacles(poor, cfg, corr);

[c, b] = computeConfidence(corr, p, poor, cfg);
verifyEqual(tc, c, 1.0);
verifyTrue(tc, b.ablated);
end

function testSafeStopReachesZeroSpeed(tc)
[~, cfg, ego] = straightSetup();
traj = safeStopTrajectory(ego, [], cfg, false);
verifyEqual(tc, traj.speed(end), 0, 'AbsTol', 1e-9);
verifyTrue(tc, traj.isSafeStop);
end

function testSafeStopDistanceIsPhysical(tc)
% Stopping distance must match v^2 / (2a) for the configured deceleration.
[~, cfg, ~] = straightSetup();
ego = makeEgoState([0 0], 0, 10.0);
traj = safeStopTrajectory(ego, [], cfg, false);
expected = 10.0^2 / (2 * cfg.ego.maxDecel);
verifyEqual(tc, traj.s(end), expected, 'RelTol', 0.05);
end

function testPlannerReturnsValidTrajectoryOnClearRoad(tc)
[g, cfg, ego] = straightSetup();
out = irpscPlanner(g, ego, [], cfg, []);

verifyTrue(tc, out.traj.valid);
verifyEqual(tc, out.status, 'ok');
verifyFalse(tc, out.isSafeStop);
verifyGreaterThan(tc, out.traj.s(end), 5);
end

function testPlannerSafeStopsOnBlockedGrid(tc)
cfg = irpscConfig('urban');
g = makeOccupancyGrid(300, 300, 0.2, [-10 -30]);
g.occ(:) = true;
ego = makeEgoState([5 0], 0, 6.0);

out = irpscPlanner(g, ego, [], cfg, []);
verifyTrue(tc, out.isSafeStop);
verifyEqual(tc, out.status, 'no_corridor');
end

function testPlannerStateRoundTrip(tc)
% Feeding the state forward must not error and must remain usable.
[g, cfg, ego] = straightSetup();
out1 = irpscPlanner(g, ego, [], cfg, []);
out2 = irpscPlanner(g, ego, [], cfg, out1.state);
verifyTrue(tc, out2.traj.valid);
end

function testBaselineAndIrpscHaveSameInterface(tc)
% The two planners must be drop-in interchangeable, or the comparison is
% not measuring what it claims to measure.
[g, cfg, ego] = straightSetup();
a = irpscPlanner(g, ego, [], cfg, []);
b = baselinePlanner(g, ego, [], cfg, []);

required = {'traj','corridor','preds','confidence','risk','ttc', ...
            'status','isSafeStop','score','state'};
for i = 1:numel(required)
    verifyTrue(tc, isfield(a, required{i}), ...
               sprintf('irpscPlanner missing field %s', required{i}));
    verifyTrue(tc, isfield(b, required{i}), ...
               sprintf('baselinePlanner missing field %s', required{i}));
end
end
