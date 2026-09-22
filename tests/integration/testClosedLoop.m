function tests = testClosedLoop
%TESTCLOSEDLOOP Integration tests for the full closed-loop simulation.
%
%   COMPONENT STATUS: REAL
%
%   Run with:  results = runtests('testClosedLoop')
%
%   These tests check that the whole chain runs end to end and that its
%   most important structural properties hold. They deliberately do NOT
%   assert particular performance numbers -- asserting "clearance must
%   exceed 0.5 m" would bake in a result nobody has measured yet. What they
%   assert instead is REPRODUCIBILITY, interface compatibility and internal
%   consistency, which are properties, not results.
%
%   These tests are slower than the unit tests because each one runs a full
%   simulation. Durations are kept short deliberately.
%
%   NOT YET EXECUTED. MATLAB is not installed on the machine where this was
%   written. Run on the destination laptop and report the real result.
%
%   Requires: base MATLAB only.

tests = functiontests(localfunctions);
end

% =====================================================================
function testScenariosAllBuild(tc)
names = {'village','urban','highway','market','cattle'};
for i = 1:numel(names)
    scn = buildScenario(names{i}, 1);
    verifyTrue(tc, isfield(scn,'grid'), sprintf('%s has no grid', names{i}));
    verifyTrue(tc, isfield(scn,'actors'), sprintf('%s has no actors', names{i}));
    verifyNotEmpty(tc, scn.sihScenario);
    verifyNotEmpty(tc, scn.description);
    % The ego must start somewhere drivable, or the run is meaningless.
    verifyFalse(tc, isOccupiedAt(scn.grid, scn.egoStart.pos), ...
                sprintf('%s: ego start is not in drivable space', names{i}));
end
end

function testUnknownScenarioErrors(tc)
verifyError(tc, @() buildScenario('not_a_scenario', 1), ...
            'buildScenario:unknownScenario');
end

function testShortRunCompletesWithoutError(tc)
opts = struct('seed', 1, 'maxTime', 4.0);
[log, M] = runScenario('village', @irpscPlanner, [], opts);
verifyNotEmpty(tc, log.t);
verifyTrue(tc, isstruct(M));
verifyEqual(tc, numel(log.t), numel(log.egoSpeed));
end

function testRunIsReproducibleWithSameSeed(tc)
% This property underpins the entire baseline comparison. If two runs with
% the same seed diverge, no comparison in this project means anything.
opts = struct('seed', 7, 'maxTime', 4.0);
log1 = runScenario('village', @irpscPlanner, [], opts);
log2 = runScenario('village', @irpscPlanner, [], opts);

verifyEqual(tc, numel(log1.t), numel(log2.t));
verifyEqual(tc, log1.egoPos, log2.egoPos, 'AbsTol', 1e-12);
verifyEqual(tc, log1.egoSpeed, log2.egoSpeed, 'AbsTol', 1e-12);
end

function testDifferentSeedsGiveDifferentNoise(tc)
% Sanity check the opposite direction: the seed must actually do something,
% otherwise the reproducibility test above would pass trivially.
log1 = runScenario('village', @irpscPlanner, [], struct('seed',1,'maxTime',4.0));
log2 = runScenario('village', @irpscPlanner, [], struct('seed',99,'maxTime',4.0));
verifyNotEqual(tc, log1.nDetections, log2.nDetections);
end

function testBothPlannersRunOnAllScenarios(tc)
names = {'village','urban','highway','market','cattle'};
for i = 1:numel(names)
    opts = struct('seed', 3, 'maxTime', 3.0);
    [logA, ~] = runScenario(names{i}, @irpscPlanner, [], opts);
    [logB, ~] = runScenario(names{i}, @baselinePlanner, [], opts);
    verifyNotEmpty(tc, logA.t, sprintf('IR-PSC produced no log on %s', names{i}));
    verifyNotEmpty(tc, logB.t, sprintf('baseline produced no log on %s', names{i}));
end
end

function testMetricsAreComputedNotInvented(tc)
% computeMetrics must refuse to produce anything from an empty log.
cfg = irpscConfig();
verifyError(tc, @() computeMetrics(struct(), cfg), 'computeMetrics:emptyLog');
end

function testMetricsProvenanceRecorded(tc)
opts = struct('seed', 2, 'maxTime', 3.0);
[~, M] = runScenario('village', @irpscPlanner, [], opts);
verifyTrue(tc, isfield(M,'provenance'));
verifyFalse(tc, M.provenance.realWorldClaim);
verifyEqual(tc, M.provenance.source, 'simulation');
end

function testAblationSwitchesChangeBehaviour(tc)
% If disabling a component changes nothing, the ablation study would be
% meaningless. This checks the switches are actually wired up.
cfgFull = irpscConfig('market');
cfgNoPred = cfgFull;
cfgNoPred.ablation.usePrediction = false;

opts = struct('seed', 5, 'maxTime', 6.0);
logFull = runScenario('market', @irpscPlanner, cfgFull,   opts);
logNo   = runScenario('market', @irpscPlanner, cfgNoPred, opts);

verifyNotEqual(tc, logFull.egoPos, logNo.egoPos);
end

function testPerfectPerceptionBypassWorks(tc)
opts = struct('seed', 4, 'maxTime', 3.0, 'usePerfectPerception', true);
[log, ~] = runScenario('village', @irpscPlanner, [], opts);
verifyTrue(tc, log.perfectPerception);
verifyNotEmpty(tc, log.t);
end

function testLoggedArraysAreConsistentLength(tc)
opts = struct('seed', 6, 'maxTime', 4.0);
log = runScenario('urban', @irpscPlanner, [], opts);
n = numel(log.t);
verifyEqual(tc, size(log.egoPos,1), n);
verifyEqual(tc, numel(log.egoSpeed), n);
verifyEqual(tc, numel(log.state), n);
verifyEqual(tc, numel(log.status), n);
verifyEqual(tc, numel(log.traj), n);
verifyEqual(tc, numel(log.collided), n);
end

function testTrackerProducesConfirmedTracks(tc)
% End-to-end check that the perception chain yields usable tracks: with a
% scenario full of road users, at least one confirmed track should exist
% after enough frames for confirmation.
opts = struct('seed', 8, 'maxTime', 6.0);
log = runScenario('urban', @irpscPlanner, [], opts);

found = false;
for k = 1:numel(log.tracks)
    if ~isempty(log.tracks{k})
        found = true;
        break;
    end
end
verifyTrue(tc, found, 'No confirmed tracks were ever produced');
end
