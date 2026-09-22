function [action, dstate] = decisionLogic(plan, cfg, dstate)
%DECISIONLOGIC Six-state supervisory decision logic with debounce.
%
%   COMPONENT STATUS: REAL (MATLAB implementation)
%   RELATIONSHIP TO STATEFLOW: this file is the executable specification of
%   the Stateflow chart described in decision/stateflow/STATEFLOW_SPEC.md.
%   No .sfx or .slx artefact exists in this package, because none could be
%   created or verified without MATLAB and Stateflow installed. Fabricating
%   one would be dishonest. On the destination laptop, either build the
%   chart from the specification, or keep using this function -- it is a
%   complete, working implementation, labelled DOCUMENTED FALLBACK when used
%   in place of Stateflow. See docs/COMPONENT_REGISTER.md.
%
%   [action, dstate] = DECISIONLOGIC(plan, cfg, dstate) maps the planner's
%   output onto a driving mode and a speed limit.
%
%   STATES
%   ------
%     NORMAL_DRIVING       Clear road, good confidence. Full speed allowed.
%     HAZARD_ASSESSMENT    Something is developing. Mild speed reduction
%                          while the situation is watched. This state exists
%                          so the system reacts EARLY and gently instead of
%                          jumping straight to hard avoidance.
%     PREDICTIVE_AVOIDANCE Risk is real. The deformed trajectory is being
%                          driven, at reduced speed.
%     CONSERVATIVE_DRIVING Confidence is low. The inputs are not trustworthy
%                          enough for normal operation, so speed is capped
%                          hard regardless of whether a hazard is visible.
%     SAFE_STOP            No feasible path, or risk beyond the stop
%                          threshold. Bring the vehicle to a standstill.
%     RECOVERY             Conditions have improved after a stop or a
%                          conservative episode. A deliberate, observable
%                          transition back rather than an instant snap to
%                          normal.
%
%   WHY DEBOUNCE IS NOT OPTIONAL
%   ----------------------------
%   Perception output is noisy. A confidence value hovering at the threshold
%   would, without hysteresis, flip the state every frame, and the vehicle
%   would surge and brake repeatedly -- worse and less predictable than
%   either mode alone. Three mechanisms prevent that:
%
%     1. Asymmetric debounce. Entering a more cautious state needs
%        cfg.decision.debounceEnter consecutive frames (default 3, fast).
%        Leaving one needs cfg.decision.debounceExit frames (default 8,
%        slow). The system becomes careful quickly and relaxes reluctantly.
%
%     2. Minimum dwell. No state may be left within
%        cfg.decision.minDwellFrames of entering it.
%
%     3. Emergency override. A safe stop bypasses both, because a debounce
%        counter must never delay an emergency response. This is the one
%        deliberate exception and it only ever escalates, never relaxes.
%
%   PHASE 2 CHANGES
%   ---------------
%     - EMERGENCY is judged on the PLANNED trajectory (plan.ttcPlanned), not
%       on the do-nothing TTC. Previously a hazard the planner had already
%       steered around, or a truck it was already following, still forced a
%       safe stop. The do-nothing TTC (plan.ttc) still escalates to
%       HAZARD_ASSESSMENT, so danger is never hidden by a successful dodge.
%       Plans without a ttcPlanned field fall back to plan.ttc.
%     - action.emergencyBrake distinguishes a controlled stop (service
%       braking) from genuine emergency braking. It is true only when the
%       planner reports that service braking is insufficient
%       (plan.emergencyBrake) or the planned-path conflict is imminent.
%     - Planner behaviour flags (following, yielding, stopping before a
%       blocked passage, potholes, narrow passage, deformation) raise at
%       least HAZARD_ASSESSMENT and give the operator a readable reason.
%     - Speed-cap factors moved to cfg.decision.
%
%   Inputs:
%       plan   - planner output struct from irpscPlanner() or baselinePlanner()
%       cfg    - config struct from irpscConfig()
%       dstate - decision state from the previous call, or [] to initialise
%
%   Outputs:
%       action - struct with fields:
%                .state        char, current state name
%                .speedLimit   m/s cap to apply to the trajectory
%                .useSafeStop  logical, command a safe stop
%                .emergencyBrake logical, brake beyond service deceleration
%                .reason       char, human-readable trigger, for demos/logs
%                .stateChanged logical, true on the frame the state changed
%       dstate - updated decision state, pass into the next call
%
%   Example:
%       dstate = [];
%       [action, dstate] = decisionLogic(plan, cfg, dstate);
%
%   Requires: base MATLAB only.
%
%   See also IRPSCPLANNER, COMPUTECONFIDENCE, STATEFLOW_SPEC.md.

STATES = {'NORMAL_DRIVING','HAZARD_ASSESSMENT','PREDICTIVE_AVOIDANCE', ...
          'CONSERVATIVE_DRIVING','SAFE_STOP','RECOVERY'};

% --- Initialise --------------------------------------------------------
if isempty(dstate) || ~isstruct(dstate)
    dstate.state          = 'NORMAL_DRIVING';
    dstate.framesInState  = 0;
    dstate.enterCounter   = 0;
    dstate.exitCounter    = 0;
    dstate.candidate      = '';
    dstate.goodFrames     = 0;
    dstate.frameCount     = 0;
    dstate.history        = {};
end

d = cfg.decision;
dstate.frameCount    = dstate.frameCount + 1;
dstate.framesInState = dstate.framesInState + 1;
prevState = dstate.state;

% --- Gather the evidence ------------------------------------------------
risk       = getOr(plan, 'risk', 0);
conf       = getOr(plan, 'confidence', 1);
ttc        = getOr(plan, 'ttc', Inf);                  % nominal, follows the road
ttcPlan    = getOr(plan, 'ttcPlanned', ttc);           % along the planned trajectory
planEmerg  = getOr(plan, 'emergencyBrake', false);
beh        = getOr(plan, 'behaviour', struct());
isSafeStop = getOr(plan, 'isSafeStop', false);
feasible   = getOr(plan, 'feasible', true);
clearOk    = getOr(plan, 'clearanceOk', true);
status     = getOr(plan, 'status', 'ok');

% --- Emergency: bypass all debounce -------------------------------------
emergency = isSafeStop || ~feasible || ~clearOk || planEmerg || ...
            risk >= d.riskStop || ttcPlan <= cfg.risk.ttcCritical || ...
            any(strcmp(status, {'no_corridor','deformation_infeasible', ...
                                'clearance_failed','feasibility_failed', ...
                                'no_feasible_candidate'}));

if emergency
    reason = emergencyReason(status, risk, ttcPlan, feasible, clearOk, d, cfg);
    brakeHard = planEmerg || ttcPlan <= cfg.risk.ttcCritical;
    % Emergency BRAKING only means something while moving. A stopped
    % vehicle with a road user approaching it stays in SAFE_STOP, but is
    % not reported as braking hard.
    trajNow = getOr(plan, 'traj', struct());
    if isstruct(trajNow) && isfield(trajNow, 'speed') && ~isempty(trajNow.speed) && ...
            trajNow.speed(1) < 0.3
        brakeHard = false;
    end
    if brakeHard
        reason = ['EMERGENCY BRAKE: ' reason];
    end
    dstate = enterState(dstate, 'SAFE_STOP');
    action = makeAction('SAFE_STOP', 0, true, reason, ...
                        ~strcmp(prevState,'SAFE_STOP'), brakeHard);
    dstate.goodFrames = 0;
    dstate.history{end+1} = 'SAFE_STOP';
    return;
end

% --- Determine the state conditions would call for ----------------------
if conf < d.confLow
    desired = 'CONSERVATIVE_DRIVING';
    reason  = sprintf('confidence %.2f below %.2f', conf, d.confLow);
elseif risk >= d.riskAvoid || (flagOf(beh, 'avoiding') && risk >= d.riskHazard)
    desired = 'PREDICTIVE_AVOIDANCE';
    if risk >= d.riskAvoid
        reason = sprintf('risk %.2f at or above %.2f', risk, d.riskAvoid);
    else
        reason = sprintf('deforming around predicted hazard (risk %.2f)', risk);
    end
elseif risk >= d.riskHazard || ttc <= cfg.risk.ttcWarning || hazardFlag(beh)
    desired = 'HAZARD_ASSESSMENT';
    if risk >= d.riskHazard
        reason = sprintf('risk %.2f at or above %.2f', risk, d.riskHazard);
    elseif ttc <= cfg.risk.ttcWarning
        reason = sprintf('time to conflict %.1f s at or below %.1f s', ...
                         ttc, cfg.risk.ttcWarning);
    else
        reason = getOr(beh, 'reason', 'planner hazard response');
    end
elseif conf < d.confHigh
    desired = 'CONSERVATIVE_DRIVING';
    reason  = sprintf('confidence %.2f below %.2f', conf, d.confHigh);
else
    desired = 'NORMAL_DRIVING';
    reason  = 'clear road, confidence sufficient';
end

% --- Recovery gate ------------------------------------------------------
% After a stop or a conservative episode, require a sustained run of good
% frames before normal driving is permitted again.
conditionsGood = conf >= d.confHigh && risk < d.riskHazard && ...
                 ttc > cfg.risk.ttcWarning && ~hazardFlag(beh);
if conditionsGood
    dstate.goodFrames = dstate.goodFrames + 1;
else
    dstate.goodFrames = 0;
end

leavingCaution = any(strcmp(prevState, {'SAFE_STOP','CONSERVATIVE_DRIVING', ...
                                        'PREDICTIVE_AVOIDANCE'}));
if leavingCaution && strcmp(desired,'NORMAL_DRIVING')
    if dstate.goodFrames < d.recoveryFrames
        desired = 'RECOVERY';
        reason  = sprintf('recovering: %d of %d good frames', ...
                          dstate.goodFrames, d.recoveryFrames);
    end
end
if strcmp(prevState,'RECOVERY') && strcmp(desired,'NORMAL_DRIVING') && ...
        dstate.goodFrames < d.recoveryFrames
    desired = 'RECOVERY';
    reason  = sprintf('recovering: %d of %d good frames', ...
                      dstate.goodFrames, d.recoveryFrames);
end

% --- Debounce -----------------------------------------------------------
if strcmp(desired, prevState)
    dstate.enterCounter = 0;
    dstate.exitCounter  = 0;
    dstate.candidate    = '';
    newState = prevState;
else
    if ~strcmp(dstate.candidate, desired)
        dstate.candidate    = desired;
        dstate.enterCounter = 1;
        dstate.exitCounter  = 1;
    else
        dstate.enterCounter = dstate.enterCounter + 1;
        dstate.exitCounter  = dstate.exitCounter + 1;
    end

    escalating = severity(desired, STATES) > severity(prevState, STATES);
    if escalating
        needed = d.debounceEnter;      % become careful quickly
    else
        needed = d.debounceExit;       % relax slowly
    end

    dwellOk = dstate.framesInState >= d.minDwellFrames;

    if dstate.enterCounter >= needed && dwellOk
        newState = desired;
    else
        newState = prevState;
        reason   = sprintf('%s (debouncing toward %s: %d of %d)', ...
                           reason, desired, dstate.enterCounter, needed);
    end
end

if ~strcmp(newState, prevState)
    dstate = enterState(dstate, newState);
end

% --- Map the state onto a speed limit -----------------------------------
switch newState
    case 'NORMAL_DRIVING'
        speedLimit  = cfg.ego.maxSpeed;
        useSafeStop = false;
    case 'HAZARD_ASSESSMENT'
        speedLimit  = d.hazardSpeedFactor * cfg.ego.maxSpeed;
        useSafeStop = false;
    case 'PREDICTIVE_AVOIDANCE'
        speedLimit  = d.avoidanceSpeedFactor * cfg.ego.maxSpeed;
        useSafeStop = false;
    case 'CONSERVATIVE_DRIVING'
        speedLimit  = min(d.conservativeSpeed, cfg.ego.maxSpeed);
        useSafeStop = false;
    case 'SAFE_STOP'
        speedLimit  = 0;
        useSafeStop = true;
    case 'RECOVERY'
        speedLimit  = min(d.recoverySpeedFactor * d.conservativeSpeed, cfg.ego.maxSpeed);
        useSafeStop = false;
    otherwise
        speedLimit  = min(d.conservativeSpeed, cfg.ego.maxSpeed);
        useSafeStop = false;
end

action = makeAction(newState, speedLimit, useSafeStop, reason, ...
                    ~strcmp(newState, prevState));
dstate.history{end+1} = newState;
end

% =====================================================================
function a = makeAction(state, speedLimit, useSafeStop, reason, changed, emergencyBrake)
if nargin < 6, emergencyBrake = false; end
a.state          = state;
a.speedLimit     = speedLimit;
a.useSafeStop    = useSafeStop;
a.reason         = reason;
a.stateChanged   = changed;
a.emergencyBrake = logical(emergencyBrake);
end

% =====================================================================
function tf = flagOf(beh, name)
tf = isstruct(beh) && isfield(beh, name) && ~isempty(beh.(name)) && logical(beh.(name));
end

% =====================================================================
function tf = hazardFlag(beh)
%HAZARDFLAG Planner is actively responding to something on the road.
names = {'following','yielding','blockedAhead','potholeSlow','potholeAvoid', ...
         'narrowPassage','avoiding'};
tf = false;
for i = 1:numel(names)
    if flagOf(beh, names{i})
        tf = true;
        return;
    end
end
end

% =====================================================================
function dstate = enterState(dstate, newState)
dstate.state         = newState;
dstate.framesInState = 0;
dstate.enterCounter  = 0;
dstate.exitCounter   = 0;
dstate.candidate     = '';
end

% =====================================================================
function s = severity(stateName, STATES)
%SEVERITY Rank states by caution, so escalation can be told from relaxation.
order = {'NORMAL_DRIVING','RECOVERY','HAZARD_ASSESSMENT', ...
         'CONSERVATIVE_DRIVING','PREDICTIVE_AVOIDANCE','SAFE_STOP'};
s = find(strcmp(order, stateName), 1);
if isempty(s)
    s = 1;
end
end

% =====================================================================
function r = emergencyReason(status, risk, ttc, feasible, clearOk, d, cfg)
if ~strcmp(status,'ok')
    r = sprintf('planner status: %s', status);
elseif ~clearOk
    r = 'clearance check failed';
elseif ~feasible
    r = 'trajectory not feasible for this vehicle';
elseif risk >= d.riskStop
    r = sprintf('risk %.2f at or above stop threshold %.2f', risk, d.riskStop);
elseif ttc <= cfg.risk.ttcCritical
    r = sprintf('time to conflict %.1f s at or below critical %.1f s', ...
                ttc, cfg.risk.ttcCritical);
else
    r = 'emergency condition';
end
end

% =====================================================================
function v = getOr(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
