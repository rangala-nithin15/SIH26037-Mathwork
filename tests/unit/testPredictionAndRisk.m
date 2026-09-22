function tests = testPredictionAndRisk
%TESTPREDICTIONANDRISK Unit tests for prediction, uncertainty and risk.
%
%   COMPONENT STATUS: REAL
%
%   Run with:  results = runtests('testPredictionAndRisk')
%
%   These tests pin down the behaviours the IR-PSC argument rests on:
%   uncertainty must GROW with horizon, agile classes must spread faster
%   than rigid ones, risk must fall with distance, and uncertainty must
%   actually widen the hazard rather than merely being reported.
%
%   NOT YET EXECUTED. MATLAB is not installed on the machine where this was
%   written. Run on the destination laptop and report the real result.
%
%   Requires: base MATLAB only.

tests = functiontests(localfunctions);
end

% =====================================================================
function testConstantVelocityStraightLine(tc)
o = makeObstacle(1, 'car', [0 0], [10 0]);
p = predictConstantVelocity(o, [0 1 2]);
verifyEqual(tc, p(:,1), [0; 10; 20], 'AbsTol', 1e-9);
verifyEqual(tc, p(:,2), [0; 0; 0],  'AbsTol', 1e-9);
end

function testConstantVelocityRespectsClassSpeedCap(tc)
% A pedestrian cannot be predicted to travel at 30 m/s.
o = makeObstacle(1, 'pedestrian', [0 0], [30 0]);
[~, v] = predictConstantVelocity(o, [0 1 2]);
verifyLessThanOrEqual(tc, max(sqrt(sum(v.^2,2))), 2.6);
end

function testUncertaintyGrowsWithHorizon(tc)
cfg = irpscConfig();
o   = makeObstacle(1, 'car', [0 0], [10 0]);
[sL, sT] = predictionUncertainty(o, [0 1 2 3], cfg);
verifyTrue(tc, all(diff(sL) > 0), 'longitudinal sigma must grow');
verifyTrue(tc, all(diff(sT) > 0), 'lateral sigma must grow');
end

function testAgileClassSpreadsFasterLaterally(tc)
% This is the encoding of "road users here do not follow lanes". A
% pedestrian must have far more lateral uncertainty than a bus.
cfg = irpscConfig();
t   = [0 1 2 3];
% Phase 2: both at the SAME speed. The original test gave the bus 8 m/s,
% so the heading-uncertainty term (distance travelled x headingStd)
% dominated and the test compared speed, not class. It had never been run.
bus = makeObstacle(1, 'bus',        [0 0], [1 0]);
ped = makeObstacle(2, 'pedestrian', [0 0], [1 0]);

[~, sBus] = predictionUncertainty(bus, t, cfg);
[~, sPed] = predictionUncertainty(ped, t, cfg);

% Compare at t = 2 s: by t = 3 s both reach the cfg.prediction.maxSigma cap
% and are equal, which the original assertion at t(end) did not allow for.
verifyGreaterThan(tc, sPed(3), sBus(3));
verifyGreaterThanOrEqual(tc, sPed(end), sBus(end));
end

function testLowConfidenceInflatesUncertainty(tc)
cfg = irpscConfig();
t   = [0 1 2];
sure   = makeObstacle(1, 'car', [0 0], [10 0], 'Confidence', 1.0, 'Age', 50);
unsure = makeObstacle(2, 'car', [0 0], [10 0], 'Confidence', 0.3, 'Age', 50);

[sSure, ~]   = predictionUncertainty(sure,   t, cfg);
[sUnsure, ~] = predictionUncertainty(unsure, t, cfg);

verifyGreaterThan(tc, sUnsure(end), sSure(end));
end

function testUncertaintyAblationFlattensSigma(tc)
cfg = irpscConfig();
cfg.ablation.useUncertainty = false;
o = makeObstacle(1, 'motorcycle', [0 0], [12 0]);
[sL, sT] = predictionUncertainty(o, [0 1 2 3], cfg);
verifyLessThan(tc, max(sL), 0.1);
verifyLessThan(tc, max(sT), 0.1);
end

function testPredictObstaclesEmptyInput(tc)
cfg = irpscConfig();
p = predictObstacles([], cfg, []);
verifyEmpty(tc, p);
end

function testPredictObstaclesShape(tc)
cfg = irpscConfig();
o   = makeObstacle(1, 'car', [10 0], [-5 0]);
p   = predictObstacles(o, cfg, []);

T = numel(0:cfg.prediction.dt:cfg.prediction.horizon);
verifySize(tc, p(1).pos, [T 2]);
verifySize(tc, p(1).sigmaLong, [T 1]);
verifyEqual(tc, p(1).id, 1);
end

function testPredictionAblationFreezesObstacles(tc)
cfg = irpscConfig();
cfg.ablation.usePrediction = false;
o = makeObstacle(1, 'car', [10 5], [-8 0]);
p = predictObstacles(o, cfg, []);
% Every predicted position must equal the current position.
verifyEqual(tc, p(1).pos, repmat([10 5], size(p(1).pos,1), 1), 'AbsTol', 1e-9);
end

function testRiskIsZeroFarAway(tc)
cfg = irpscConfig();
vp  = vehicleParams(cfg);
o   = makeObstacle(1, 'car', [200 200], [0 0]);
p   = predictObstacles(o, cfg, []);
r   = predictedOccupancyRisk([0 0], 0, p, vp.discRadius, cfg);
verifyEqual(tc, r, 0, 'AbsTol', 1e-9);
end

function testRiskIsHighAtObstacleCentre(tc)
cfg = irpscConfig();
vp  = vehicleParams(cfg);
o   = makeObstacle(1, 'car', [10 0], [0 0]);
p   = predictObstacles(o, cfg, []);
r   = predictedOccupancyRisk([10 0], 0, p, vp.discRadius, cfg);
verifyGreaterThan(tc, r, 0.5);
end

function testRiskDecreasesWithDistance(tc)
cfg = irpscConfig();
vp  = vehicleParams(cfg);
o   = makeObstacle(1, 'car', [10 0], [0 0]);
p   = predictObstacles(o, cfg, []);

rNear = predictedOccupancyRisk([10 0],   0, p, vp.discRadius, cfg);
rMid  = predictedOccupancyRisk([10 2.5], 0, p, vp.discRadius, cfg);
rFar  = predictedOccupancyRisk([10 8],   0, p, vp.discRadius, cfg);

verifyGreaterThan(tc, rNear, rMid);
verifyGreaterThanOrEqual(tc, rMid, rFar);
end

function testRiskAlwaysInUnitInterval(tc)
% Combining many overlapping hazards must never exceed 1.
cfg = irpscConfig();
vp  = vehicleParams(cfg);
obs = makeObstacle(1, 'car', [10 0], [0 0]);
for k = 2:6
    obs(k) = makeObstacle(k, 'car', [10 0], [0 0]);
end
p = predictObstacles(obs, cfg, []);
r = predictedOccupancyRisk([10 0], 0, p, vp.discRadius, cfg);
verifyLessThanOrEqual(tc, r, 1);
verifyGreaterThanOrEqual(tc, r, 0);
end

function testVulnerableClassScoresHigherRisk(tc)
% A pedestrian at the same place as a car must produce more risk, because
% the class weight favours protecting vulnerable road users.
cfg = irpscConfig();
vp  = vehicleParams(cfg);

car = makeObstacle(1, 'car',        [10 0], [0 0]);
ped = makeObstacle(2, 'pedestrian', [10 0], [0 0]);

pc = predictObstacles(car, cfg, []);
pp = predictObstacles(ped, cfg, []);

rCar = predictedOccupancyRisk([10.8 0], 0, pc, vp.discRadius, cfg);
rPed = predictedOccupancyRisk([10.8 0], 0, pp, vp.discRadius, cfg);

verifyGreaterThan(tc, rPed, rCar);
end

function testTimeToConflictHeadOn(tc)
% Ego at origin travelling +x at 10 m/s; obstacle 30 m ahead, stationary.
% Contact happens when the gap closes, so TTC is under 3 s.
cfg = irpscConfig();
vp  = vehicleParams(cfg);
ego = makeEgoState([0 0], 0, 10.0);
o   = makeObstacle(1, 'car', [30 0], [0 0]);
p   = predictObstacles(o, cfg, []);

[ttc, id] = timeToConflict(ego, p, cfg, vp);
verifyLessThan(tc, ttc, 3.0);
verifyGreaterThan(tc, ttc, 1.0);
verifyEqual(tc, id, 1);
end

function testTimeToConflictNoObstacles(tc)
cfg = irpscConfig();
vp  = vehicleParams(cfg);
ego = makeEgoState([0 0], 0, 10.0);
ttc = timeToConflict(ego, [], cfg, vp);
verifyEqual(tc, ttc, Inf);
end

function testTimeToConflictClearPath(tc)
% Obstacle far to the side: no conflict predicted.
cfg = irpscConfig();
vp  = vehicleParams(cfg);
ego = makeEgoState([0 0], 0, 10.0);
o   = makeObstacle(1, 'car', [20 50], [0 0]);
p   = predictObstacles(o, cfg, []);
ttc = timeToConflict(ego, p, cfg, vp);
verifyEqual(tc, ttc, Inf);
end
