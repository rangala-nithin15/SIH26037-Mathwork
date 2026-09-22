function cfg = irpscConfig(profile)
%IRPSCCONFIG Master configuration for the IR-PSC planning system.
%
%   COMPONENT STATUS: REAL
%
%   cfg = IRPSCCONFIG() returns the default ('urban') configuration.
%   cfg = IRPSCCONFIG(profile) returns a scenario-tuned configuration.
%
%   Every tunable number in the IR-PSC pipeline lives here. Nothing else in
%   the project should contain a magic constant. This makes ablation studies
%   straightforward: an ablation is just a modified copy of this struct.
%
%   Inputs:
%       profile - char, one of 'urban' (default), 'village', 'highway',
%                 'market', 'cattle'. Case-insensitive.
%
%   Outputs:
%       cfg - struct with fields .ego .corridor .prediction .risk .deform
%             .safety .traj .decision .control .sim .ablation .profile
%
%   Example:
%       cfg = irpscConfig('village');
%
%   Requires: base MATLAB only.
%
%   See also VEHICLEPARAMS, OBJECTCLASSES, IRPSCPLANNER.

if nargin < 1 || isempty(profile)
    profile = 'urban';
end
profile = lower(char(profile));

% ---------------------------------------------------------------------
% Ego vehicle: a representative compact Indian passenger car.
% SOURCE: nominal values chosen for simulation. NOT measured from a real
% vehicle. Replace with measured values before making any physical claim.
% ---------------------------------------------------------------------
cfg.ego.length         = 3.99;   % m, overall length
cfg.ego.width          = 1.68;   % m, overall width
cfg.ego.wheelbase      = 2.45;   % m
cfg.ego.rearOverhang   = 0.70;   % m, rear axle to rear bumper
cfg.ego.maxSpeed       = 16.7;   % m/s (~60 km/h)
cfg.ego.maxAccel       = 2.0;    % m/s^2, comfortable acceleration
cfg.ego.maxDecel       = 4.5;    % m/s^2, service braking magnitude
cfg.ego.emergencyDecel = 7.0;    % m/s^2, emergency braking magnitude
cfg.ego.maxSteer       = 0.61;   % rad (~35 deg) road-wheel angle
cfg.ego.maxSteerRate   = 0.50;   % rad/s
cfg.ego.maxLatAccel    = 3.0;    % m/s^2, comfort limit
cfg.ego.maxJerk        = 3.0;    % m/s^3, COMFORT jerk (profile shaping, reported)
cfg.ego.brakeJerk      = 8.0;    % m/s^3, how fast brake force can physically build (assumption)

% ---------------------------------------------------------------------
% Corridor extraction. Drivable space is the PRIMARY planning constraint;
% lane markings are treated as an optional cue only.
% ---------------------------------------------------------------------
cfg.corridor.lookaheadDist    = 40.0;  % m, corridor length ahead of ego
cfg.corridor.stationStep      = 1.0;   % m, spacing of corridor stations
cfg.corridor.maxRayLength     = 12.0;  % m, max lateral search for a boundary
cfg.corridor.rayStep          = 0.10;  % m, ray-march resolution
cfg.corridor.minWidth         = 2.6;   % m, below this the corridor is unusable
cfg.corridor.seedHeadingFan   = 0.70;  % rad, half-angle of forward search fan
cfg.corridor.seedFanCount     = 21;    % headings tested per seed step
cfg.corridor.centerlineSmooth = 5;     % moving-average half-window (stations)
cfg.corridor.boundaryErode    = 0.15;  % m, shrink boundaries inward
cfg.corridor.seedTurnPenalty  = 3.0;   % m of free distance traded per rad of turn
cfg.corridor.seedProbeLength  = 15.0;  % m, how far each seed candidate looks ahead
% Narrow-passage handling. A station narrower than minWidth is NARROW (the
% vehicle may pass slowly with reduced side margins); a station narrower
% than the body plus the hard minimum clearance is BLOCKED. A blocked
% station no longer invalidates the whole corridor: the corridor is
% truncated there and the vehicle is planned to stop before it.
cfg.corridor.narrowSpeed      = 3.0;   % m/s, speed cap through narrow stations
cfg.corridor.minUsableLength  = 4.0;   % m, usable corridor shorter than this is invalid
cfg.corridor.stopStandoff     = 2.5;   % m, front bumper stops this far before a blockage

% ---------------------------------------------------------------------
% Short-term prediction
% ---------------------------------------------------------------------
cfg.prediction.horizon     = 3.0;   % s, prediction horizon
cfg.prediction.dt          = 0.20;  % s, prediction time step
cfg.prediction.posNoiseStd = 0.30;  % m, initial position std dev
cfg.prediction.velNoiseStd = 0.40;  % m/s, initial velocity std dev
cfg.prediction.accelStd    = 1.20;  % m/s^2, process noise (drives growth)
cfg.prediction.headingStd  = 0.15;  % rad, heading std dev for classes not listed below
% Heading uncertainty per class (Phase 2). Vehicles follow the road, so a
% sustained 0.15 rad heading error over 3 s (2.7 m of lateral sigma for a
% car at 9 m/s) made every oncoming car look like a certain collision.
% People and animals change direction freely. Engineering judgement.
cfg.prediction.headingStdClass = struct( ...
    'car', 0.04, 'bus', 0.03, 'truck', 0.03, 'motorcycle', 0.08, ...
    'bicycle', 0.08, 'autorickshaw', 0.06, 'pedestrian', 0.25, ...
    'pushcart', 0.10, 'animal', 0.30, 'unknown', 0.15);
cfg.prediction.maxSigma    = 6.0;   % m, cap on predicted std dev

% Irregular-motion allowance: EXTRA lateral uncertainty per class, expressed
% in metres of std dev added per second of horizon. On unstructured Indian
% roads, road users do not follow lanes, so lateral spread must grow faster
% for agile and erratic classes than a lane-following model would assume.
cfg.prediction.lateralSpread = struct( ...
    'car',          0.25, ...
    'bus',          0.15, ...
    'truck',        0.15, ...
    'motorcycle',   0.70, ...
    'bicycle',      0.55, ...
    'autorickshaw', 0.55, ...
    'pedestrian',   0.90, ...
    'pushcart',     0.45, ...
    'animal',       1.10, ...
    'unknown',      0.80);

% ---------------------------------------------------------------------
% Risk / conflict model
% ---------------------------------------------------------------------
cfg.risk.sigmaScale      = 1.0;   % multiplier on predicted uncertainty
cfg.risk.nSigmaOccupancy = 2.0;   % occupancy footprint = mean + n*sigma
cfg.risk.ttcCritical     = 1.5;   % s, below this TTC counts as critical
cfg.risk.ttcWarning      = 3.5;   % s, below this TTC counts as a warning
cfg.risk.riskFloor       = 1e-4;  % ignore risk contributions below this
cfg.risk.gridOffsetStep  = 0.15;  % m, lateral resolution of the risk grid (finer => less smoothing needed)
cfg.risk.ttcSigmaFactor  = 0.25;  % std devs of predicted uncertainty added to TTC footprint
                                  % (Phase 2 tuning: 0.5 made every oncoming vehicle passing ~1 m away
                                  %  in its own lane an 'imminent conflict'; uncertainty is carried by the risk model)
cfg.risk.speedReduction  = 0.75;  % fraction of target speed removed at risk = 1
cfg.risk.beyondHorizonWeight = 0.3; % weight of risk at stations reached after the horizon

% Vulnerable road users are weighted higher so the planner leaves more room.
% This is a DESIGN CHOICE reflecting Indian-road priorities. It is not a
% validated safety guarantee and confers no safety certification.
cfg.risk.classWeight = struct( ...
    'car',          1.00, ...
    'bus',          1.10, ...
    'truck',        1.10, ...
    'motorcycle',   1.15, ...
    'bicycle',      1.30, ...
    'autorickshaw', 1.10, ...
    'pedestrian',   1.60, ...
    'pushcart',     1.25, ...
    'animal',       1.50, ...
    'unknown',      1.30);

% ---------------------------------------------------------------------
% Trajectory deformation
% ---------------------------------------------------------------------
cfg.deform.maxLateralShift = 2.5;   % m, max deviation from corridor centre
cfg.deform.influenceRadius = 6.0;   % m, longitudinal reach of one hazard
cfg.deform.gain            = 1.0;   % overall deformation strength
cfg.deform.iterations      = 3;     % relaxation sweeps
cfg.deform.boundaryMargin  = 0.35;  % m, keep clear of corridor edges
cfg.deform.temporalAlpha   = 0.65;  % temporal blend (0 = frozen, 1 = no memory)
cfg.deform.minBoundaryMargin = 0.10; % m, margin allowed in NARROW stations
% Dynamic-programming cost weights (see deformTrajectory.m)
cfg.deform.wRisk     = 12.0;  % predicted-occupancy risk
cfg.deform.wDev      = 0.35;  % deviation from the corridor centre
cfg.deform.wSmooth   = 2.5;   % lateral change between stations
cfg.deform.wAnchor   = 4.0;   % first station stays at the vehicle
cfg.deform.wTemporal = 1.5;   % agreement with the previous plan
cfg.deform.wPothole  = 1.0;   % multiplier on pothole traversal cost
cfg.deform.wStatic   = 20.0;  % static clearance below the preferred side clearance
cfg.deform.smoothRefSpeed = 4.0; % m/s; above this wSmooth grows with (v/ref)^2
% Traffic-side preference (an OPTIONAL cue, like a lane marking): on a
% two-way road India keeps LEFT, so the preferred lateral position is this
% far left of the drivable-corridor centre, clipped to the corridor. 0 =
% plan on the centre of free space (single-track / one-way roads).
cfg.deform.preferredOffset = 0.0;  % m, positive = left

% ---------------------------------------------------------------------
% Safety thresholds
% ---------------------------------------------------------------------
cfg.safety.lateralClearance    = 0.50;  % m, desired side clearance
cfg.safety.minLateralClearance = 0.25;  % m, hard minimum
cfg.safety.longitudinalGap     = 2.0;   % m, standoff at zero speed
cfg.safety.timeGap             = 1.2;   % s, speed-dependent following gap
cfg.safety.maxCurvature        = 0.25;  % 1/m, tightest allowed path curvature
cfg.safety.minCorridorWidth    = 2.6;   % m, must exceed ego width + margins
cfg.safety.clearanceSearch     = 3.0;   % m, grid search radius around the footprint
cfg.safety.clearanceHorizon    = 2.0;   % s, hard geometric check vs predicted road users up to here
cfg.safety.yieldStandoff       = 3.0;   % m, stop this far before a predicted conflict
cfg.safety.leadDecel           = 2.5;   % m/s^2, planned decel when closing on a lead
cfg.safety.emergencyMargin     = 1.0;   % factor on maxDecel above which braking is emergency

% ---------------------------------------------------------------------
% Trajectory generation and smoothing
% ---------------------------------------------------------------------
cfg.traj.numPoints        = 41;    % samples in the output trajectory
cfg.traj.smoothWindow     = 3;     % moving-average half-window (samples)
cfg.traj.smoothPasses     = 2;     % repeated smoothing passes
cfg.traj.minSpeed         = 0.0;   % m/s
cfg.traj.speedComfortJerk = 1.5;   % m/s^3 (legacy field, see profileJerk)
% Jerk-limited speed profile (see speedProfile.m). The profile is generated
% by integrating a jerk-limited longitudinal model in time, so the planned
% speeds respect maxAccel, maxDecel AND maxJerk by construction instead of
% being rejected afterwards by checkFeasibility.
cfg.traj.profileJerk      = 2.0;   % m/s^3, jerk used to shape the profile (< ego.maxJerk: sampling margin)
cfg.traj.profileDecel     = 3.0;   % m/s^2, planned braking (< ego.maxDecel)
cfg.traj.profileGain      = 1.2;   % 1/s, speed-tracking gain of the generator
cfg.traj.profileDt        = 0.05;  % s, generator integration step
cfg.traj.profileMaxTime   = 30.0;  % s, generator time cap
cfg.traj.ceilingWindow    = 2;     % stations, erosion/averaging window of the speed ceiling
cfg.traj.profileLimitMargin = 0.97; % profile uses 97 % of accel/decel limits (sampling margin)
cfg.traj.latAccelMargin   = 0.90;  % profile plans to 90 % of maxLatAccel (tracking margin)
cfg.traj.feasibilityRetrySpeed = 0.6; % x maxSpeed: retry speed cap when a plan is infeasible

% ---------------------------------------------------------------------
% Scoring weights (scoreTrajectory.m) and confidence weights
% ---------------------------------------------------------------------
cfg.score.wRisk      = 10.0;
cfg.score.wDeviation = 1.0;
cfg.score.wCurvature = 2.0;
cfg.score.wJerkiness = 1.5;
cfg.score.wProgress  = 3.0;
cfg.score.wPothole   = 4.0;
cfg.confidence.wCorridor   = 0.35;
cfg.confidence.wPerception = 0.25;
cfg.confidence.wTrack      = 0.20;
cfg.confidence.wPredict    = 0.20;
cfg.confidence.matureAge   = 10;    % frames for a track to count as mature
cfg.confidence.emptySceneScore = 0.75;

% ---------------------------------------------------------------------
% Baseline planner (conventional fixed-candidate recipe)
% ---------------------------------------------------------------------
cfg.baseline.candidateOffsets = [-2.0, -1.0, 0.0, 1.0, 2.0];

% ---------------------------------------------------------------------
% Perception: fusion and tracking (FALLBACK F5)
% ---------------------------------------------------------------------
cfg.fusion.gateSigma       = 3.0;   % association gate, std devs
cfg.tracking.confirmHits   = 3;     % hits before a track is confirmed
cfg.tracking.maxMisses     = 5;     % consecutive misses before deletion
cfg.tracking.gateChi2      = 9.21;  % chi-square gate, 2 dof, 99 %
cfg.tracking.sigmaAccel    = 2.5;   % m/s^2 process noise
cfg.tracking.initVelVar    = 25;    % (m/s)^2 initial velocity variance

% ---------------------------------------------------------------------
% Potholes (first-class road hazards; see scenarios/makePothole.m,
% perception/detection/detectPotholes.m, perception/tracking/potholeTracker.m)
% ---------------------------------------------------------------------
% Severity classes by depth (m). SOURCE: engineering judgement, not a
% standard. Minor potholes are straddled or driven over at reduced speed;
% moderate ones are avoided when there is room, otherwise crossed slowly;
% severe ones are strongly avoided and crossed at crawl speed only if
% there is genuinely no way around.
cfg.pothole.depthModerate  = 0.05;   % m, depth at or above -> moderate
cfg.pothole.depthSevere    = 0.10;   % m, depth at or above -> severe
cfg.pothole.cost    = struct('minor', 0.6, 'moderate', 6.0, 'severe', 20.0);
cfg.pothole.speed   = struct('minor', 8.0, 'moderate', 4.0, 'severe', 2.0); % m/s over it
cfg.pothole.risk    = struct('minor', 0.20, 'moderate', 0.50, 'severe', 0.85);
cfg.pothole.wheelTrack     = 1.45;   % m, lateral distance between wheel centres
cfg.pothole.tyreWidth      = 0.20;   % m
cfg.pothole.slowdownLead   = 4.0;    % m, speed cap starts this far before the pothole
cfg.pothole.lateralMargin  = 0.20;   % m, extra tyre clearance when planning to avoid (tracking error)
cfg.pothole.confirmHits    = 3;      % detections before a pothole is confirmed
cfg.pothole.gate           = 1.5;    % m, association gate for pothole landmarks
cfg.pothole.maxMisses      = 1e9;    % potholes are static: never deleted once confirmed

% ---------------------------------------------------------------------
% Decision logic (see decision/decisionLogic.m and the Stateflow spec)
% ---------------------------------------------------------------------
cfg.decision.confHigh          = 0.70;  % above -> normal driving allowed
cfg.decision.confLow           = 0.40;  % below -> conservative mode
cfg.decision.riskHazard        = 0.25;  % risk above -> hazard assessment
cfg.decision.riskAvoid         = 0.50;  % risk above -> predictive avoidance
cfg.decision.riskStop          = 0.85;  % risk above -> safe stop
cfg.decision.debounceEnter     = 3;     % consecutive frames to ENTER a state
cfg.decision.debounceExit      = 8;     % consecutive frames to LEAVE a state
cfg.decision.minDwellFrames    = 5;     % minimum frames spent in any state
cfg.decision.conservativeSpeed = 4.0;   % m/s speed cap in conservative mode
cfg.decision.recoveryFrames    = 10;    % frames of good health before recovery
cfg.decision.hazardSpeedFactor    = 0.75;  % speed cap in HAZARD_ASSESSMENT, x maxSpeed
cfg.decision.avoidanceSpeedFactor = 0.55;  % speed cap in PREDICTIVE_AVOIDANCE, x maxSpeed
cfg.decision.recoverySpeedFactor  = 1.5;   % speed cap in RECOVERY, x conservativeSpeed

% ---------------------------------------------------------------------
% Controller
% ---------------------------------------------------------------------
cfg.control.lookaheadMin  = 3.0;   % m
cfg.control.lookaheadGain = 0.6;   % m per (m/s) of speed
cfg.control.kpSpeed       = 1.2;
cfg.control.kiSpeed       = 0.15;
cfg.control.iMax          = 2.0;   % integrator clamp
cfg.control.speedPreview  = 0.3;   % s, speed-profile preview for feed-forward
cfg.control.minPreviewDist = 1.0;  % m, minimum preview (pull-away from rest)

% ---------------------------------------------------------------------
% Simulation timing
% ---------------------------------------------------------------------
cfg.sim.dt        = 0.05;  % s, 20 Hz closed loop
cfg.sim.planEvery = 4;     % replan every N steps (=> 5 Hz planning)
cfg.sim.maxTime   = 60.0;  % s, per-scenario cap

% ---------------------------------------------------------------------
% Ablation switches. All true = full IR-PSC. Flipping one to false disables
% that stage so its contribution can be measured. See experiments/.
% ---------------------------------------------------------------------
cfg.ablation.usePrediction        = true;
cfg.ablation.useUncertainty       = true;
cfg.ablation.useConfidenceAware   = true;
cfg.ablation.useDeformation       = true;
cfg.ablation.useTemporalSmoothing = true;

% ---------------------------------------------------------------------
% Scenario profile overrides
% ---------------------------------------------------------------------
switch profile
    case 'urban'
        % Defaults above are tuned for the urban intersection case.
    case 'village'
        cfg.ego.maxSpeed            = 11.1;  % ~40 km/h
        cfg.corridor.maxRayLength   = 8.0;   % narrow road, boundaries close
        cfg.corridor.minWidth       = 2.8;
        cfg.deform.maxLateralShift  = 2.0;   % corridor bounds limit it further on narrow stretches
        cfg.safety.lateralClearance = 0.60;
    case 'highway'
        cfg.ego.maxSpeed           = 22.2;   % ~80 km/h
        cfg.corridor.lookaheadDist = 70.0;   % faster, so look further ahead
        cfg.prediction.horizon     = 4.0;
        cfg.deform.maxLateralShift = 3.5;
        cfg.risk.ttcWarning        = 5.0;
    case 'market'
        cfg.ego.maxSpeed            = 5.6;   % ~20 km/h
        cfg.corridor.lookaheadDist  = 20.0;
        cfg.prediction.horizon      = 2.5;
        cfg.safety.lateralClearance = 0.70;  % crowded, give more space
        cfg.decision.confLow        = 0.50;  % become conservative sooner
        cfg.decision.riskHazard     = 0.18;
    case 'cattle'
        cfg.ego.maxSpeed       = 11.1;
        cfg.prediction.horizon = 3.5;
        cfg.risk.ttcCritical   = 2.2;        % react earlier
        cfg.decision.riskStop  = 0.75;
    case 'demo'
        % Primary judge demonstration (scenarios/demo/scenarioDemo.m): a
        % mixed Indian road at village/town speeds.
        cfg.ego.maxSpeed            = 11.1;  % ~40 km/h
        cfg.corridor.lookaheadDist  = 45.0;
        cfg.corridor.maxRayLength   = 9.0;
        cfg.prediction.horizon      = 3.5;
        cfg.deform.maxLateralShift  = 2.8;
        cfg.deform.preferredOffset  = 0.9;   % two-way road: keep left
        cfg.sim.maxTime             = 90.0;
    otherwise
        error('irpscConfig:unknownProfile', ...
              'Unknown profile "%s". Use urban|village|highway|market|cattle|demo.', ...
              profile);
end

cfg.profile = profile;
end
