function tests = testDecisionAndVehicle
%TESTDECISIONANDVEHICLE Unit tests for decision logic, control and dynamics.
%
%   COMPONENT STATUS: REAL
%
%   Run with:  results = runtests('testDecisionAndVehicle')
%
%   The debounce tests matter most here. Hysteresis is easy to write and
%   easy to get subtly wrong, and a chattering state machine would make the
%   vehicle surge and brake repeatedly. These tests pin the behaviour down.
%
%   NOT YET EXECUTED. MATLAB is not installed on the machine where this was
%   written. Run on the destination laptop and report the real result.
%
%   Requires: base MATLAB only.

tests = functiontests(localfunctions);
end

% =====================================================================
function p = makePlan(risk, conf, ttc)
p.risk        = risk;
p.confidence  = conf;
p.ttc         = ttc;
p.isSafeStop  = false;
p.feasible    = true;
p.clearanceOk = true;
p.status      = 'ok';
end

% =====================================================================
function testStartsInNormalDriving(tc)
cfg = irpscConfig();
[a, ~] = decisionLogic(makePlan(0, 1.0, Inf), cfg, []);
verifyEqual(tc, a.state, 'NORMAL_DRIVING');
verifyEqual(tc, a.speedLimit, cfg.ego.maxSpeed);
end

function testEmergencyBypassesDebounce(tc)
% A critical hazard must produce SAFE_STOP on the very first frame. A
% debounce counter must never delay an emergency response.
cfg = irpscConfig();
p = makePlan(0.95, 1.0, 0.5);
[a, ~] = decisionLogic(p, cfg, []);
verifyEqual(tc, a.state, 'SAFE_STOP');
verifyTrue(tc, a.useSafeStop);
verifyEqual(tc, a.speedLimit, 0);
end

function testInfeasiblePlanTriggersSafeStop(tc)
cfg = irpscConfig();
p = makePlan(0, 1.0, Inf);
p.feasible = false;
[a, ~] = decisionLogic(p, cfg, []);
verifyEqual(tc, a.state, 'SAFE_STOP');
end

function testNoCorridorTriggersSafeStop(tc)
cfg = irpscConfig();
p = makePlan(0, 1.0, Inf);
p.status = 'no_corridor';
[a, ~] = decisionLogic(p, cfg, []);
verifyEqual(tc, a.state, 'SAFE_STOP');
end

function testDebounceDelaysNonEmergencyEscalation(tc)
% A moderate risk must NOT change state instantly; it needs debounceEnter
% consecutive frames AND the minimum dwell to have elapsed.
cfg = irpscConfig();
d = [];

% Settle in NORMAL_DRIVING past the minimum dwell.
for i = 1:cfg.decision.minDwellFrames + 1
    [a, d] = decisionLogic(makePlan(0, 1.0, Inf), cfg, d);
end
verifyEqual(tc, a.state, 'NORMAL_DRIVING');

% One frame of moderate risk must not be enough.
[a, d] = decisionLogic(makePlan(0.30, 1.0, Inf), cfg, d);
verifyEqual(tc, a.state, 'NORMAL_DRIVING');

% Sustained risk eventually escalates.
for i = 1:cfg.decision.debounceEnter + 2
    [a, d] = decisionLogic(makePlan(0.30, 1.0, Inf), cfg, d);
end
verifyEqual(tc, a.state, 'HAZARD_ASSESSMENT');
end

function testNoChatterUnderAlternatingInput(tc)
% Risk flipping either side of the threshold every frame must not produce a
% state change every frame. This is the whole reason debounce exists.
cfg = irpscConfig();
d = [];
states = cell(1,40);
for i = 1:40
    if mod(i,2) == 0
        r = cfg.decision.riskHazard + 0.02;
    else
        r = cfg.decision.riskHazard - 0.02;
    end
    [a, d] = decisionLogic(makePlan(r, 1.0, Inf), cfg, d);
    states{i} = a.state;
end

changes = 0;
for i = 2:40
    if ~strcmp(states{i}, states{i-1})
        changes = changes + 1;
    end
end
verifyLessThan(tc, changes, 6, ...
    'State changed too often under alternating input: debounce is not working');
end

function testLowConfidenceLeadsToConservative(tc)
cfg = irpscConfig();
d = [];
for i = 1:20
    [a, d] = decisionLogic(makePlan(0, 0.2, Inf), cfg, d);
end
verifyEqual(tc, a.state, 'CONSERVATIVE_DRIVING');
verifyLessThanOrEqual(tc, a.speedLimit, cfg.decision.conservativeSpeed);
end

function testRecoveryIsNotInstant(tc)
% After a hazard clears, the system must pass through RECOVERY rather than
% snapping straight back to full speed.
cfg = irpscConfig();
d = [];
for i = 1:15
    [~, d] = decisionLogic(makePlan(0.95, 1.0, 0.5), cfg, d);   % emergency
end
[a, d] = decisionLogic(makePlan(0, 1.0, Inf), cfg, d);
verifyNotEqual(tc, a.state, 'NORMAL_DRIVING');

% Sustained good conditions eventually restore normal driving.
for i = 1:cfg.decision.recoveryFrames + cfg.decision.debounceExit + 15
    [a, d] = decisionLogic(makePlan(0, 1.0, Inf), cfg, d);
end
verifyEqual(tc, a.state, 'NORMAL_DRIVING');
end

function testSpeedLimitOrderingAcrossStates(tc)
% More cautious states must never permit a higher speed.
cfg = irpscConfig();
d = [];
[aNormal, ~] = decisionLogic(makePlan(0, 1.0, Inf), cfg, d);

d2 = [];
for i = 1:20
    [aCons, d2] = decisionLogic(makePlan(0, 0.2, Inf), cfg, d2);
end
verifyLessThan(tc, aCons.speedLimit, aNormal.speedLimit);
end

% =====================================================================
function testBicycleModelStraightLine(tc)
cfg = irpscConfig();
ego = makeEgoState([0 0], 0, 10.0);
for i = 1:20
    ego = bicycleModelStep(ego, 0, 0, cfg);
end
% 20 steps at 0.05 s and 10 m/s = 1 s = 10 m.
verifyEqual(tc, ego.pos(1), 10.0, 'AbsTol', 0.15);
verifyEqual(tc, ego.pos(2), 0, 'AbsTol', 1e-6);
verifyEqual(tc, ego.speed, 10.0, 'AbsTol', 1e-9);
end

function testBicycleModelCannotReverseUnderBraking(tc)
% Braking a stationary vehicle must not push it backwards.
cfg = irpscConfig();
ego = makeEgoState([0 0], 0, 0.5);
for i = 1:50
    ego = bicycleModelStep(ego, 0, -cfg.ego.maxDecel, cfg);
end
verifyGreaterThanOrEqual(tc, ego.speed, 0);
end

function testBicycleModelSteeringRateLimited(tc)
% A large step command must be limited by maxSteerRate in one step.
cfg = irpscConfig();
ego = makeEgoState([0 0], 0, 5.0);
egoNext = bicycleModelStep(ego, cfg.ego.maxSteer, 0, cfg);
verifyLessThanOrEqual(tc, abs(egoNext.steer - ego.steer), ...
                      cfg.ego.maxSteerRate * cfg.sim.dt + 1e-9);
end

function testBicycleModelTurnRadius(tc)
% At full lock the vehicle must trace roughly its minimum turning radius.
cfg = irpscConfig();
vp  = vehicleParams(cfg);
ego = makeEgoState([0 0], 0, 3.0);
ego.steer = cfg.ego.maxSteer;

positions = zeros(120,2);
for i = 1:120
    ego = bicycleModelStep(ego, cfg.ego.maxSteer, 0, cfg);
    positions(i,:) = ego.pos;
end
k = pathCurvature(positions);
verifyEqual(tc, mean(abs(k(10:end-10))), vp.maxCurvature, 'RelTol', 0.20);
end

function testPurePursuitZeroErrorOnPath(tc)
% A vehicle exactly on a straight path must command near-zero steering.
cfg = irpscConfig();
vp  = vehicleParams(cfg);
traj.pos       = [(0:1:30).', zeros(31,1)];
traj.heading   = zeros(31,1);
traj.curvature = zeros(31,1);
traj.speed     = repmat(8,31,1);
traj.s         = (0:1:30).';
traj.times     = (0:1:30).'/8;
traj.valid     = true;

ego = makeEgoState([0 0], 0, 8.0);
[steer, info] = purePursuitControl(traj, ego, cfg, vp);
verifyEqual(tc, steer, 0, 'AbsTol', 1e-6);
verifyEqual(tc, info.crossTrackError, 0, 'AbsTol', 1e-6);
end

function testPurePursuitSteersTowardPath(tc)
% Offset to the right of the path: steering must be positive (turn left).
cfg = irpscConfig();
vp  = vehicleParams(cfg);
traj.pos       = [(0:1:30).', zeros(31,1)];
traj.heading   = zeros(31,1);
traj.curvature = zeros(31,1);
traj.speed     = repmat(8,31,1);
traj.s         = (0:1:30).';
traj.times     = (0:1:30).'/8;
traj.valid     = true;

ego = makeEgoState([0 -1.5], 0, 8.0);
steer = purePursuitControl(traj, ego, cfg, vp);
verifyGreaterThan(tc, steer, 0);
end

function testPurePursuitSaturates(tc)
cfg = irpscConfig();
vp  = vehicleParams(cfg);
% Phase 2: a densely sampled path heading straight left of the vehicle.
% The original two-point path put the pure-pursuit target 30 m away, which
% needs only 0.16 rad of steering, so the test could never pass. It had
% never been run.
yy = (0:1:30).';
traj.pos       = [zeros(31,1), yy];
traj.heading   = repmat(pi/2, 31, 1);
traj.curvature = zeros(31,1);
traj.speed     = repmat(5, 31, 1);
traj.s         = yy;
traj.times     = yy / 5;
traj.valid     = true;

ego = makeEgoState([0 0], 0, 5.0);
[steer, info] = purePursuitControl(traj, ego, cfg, vp);
verifyLessThanOrEqual(tc, abs(steer), cfg.ego.maxSteer + 1e-9);
verifyTrue(tc, info.saturated);
end

function testLongitudinalControlIntegratorClamped(tc)
% Sustained unreachable demand must not wind the integrator up without limit.
cfg = irpscConfig();
traj.pos   = [(0:1:20).', zeros(21,1)];
traj.s     = (0:1:20).';
traj.speed = repmat(50, 21, 1);          % unreachable target
traj.times = (0:1:20).'/50;

ego = makeEgoState([0 0], 0, 1.0);
cs = [];
for i = 1:200
    [a, cs] = longitudinalControl(traj, ego, 50, cfg, cs);
end
verifyLessThanOrEqual(tc, abs(cs.integral), cfg.control.iMax + 1e-9);
verifyLessThanOrEqual(tc, a, cfg.ego.maxAccel + 1e-9);
end
