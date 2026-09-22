function [lines, st] = viewerEvents(st, frame, plan, beh)
%VIEWEREVENTS Turn simulation state into a short, readable decision log.
%
%   COMPONENT STATUS: REAL (visualisation only; derives nothing new).
%
%   [lines, st] = VIEWEREVENTS(st, frame, plan, beh) returns the event
%   lines to append this frame, in the form
%
%       [DETECT]   Pedestrian ahead
%       [PREDICT]  Path conflict in 2.1 s
%       [PLAN]     Trajectory deformed around hazard
%       [DECISION] AVOID
%
%   Every line restates something the simulation actually reported: a new
%   confirmed track from the tracker, a confirmed pothole from the pothole
%   tracker, the planner's own behaviour flags, its status string, or a
%   decision-logic state change. Nothing is inferred beyond thresholds that
%   come from the planner configuration, and nothing is invented.
%
%   `st` carries the memory needed to report only CHANGES (which tracks and
%   potholes have been announced, the previous state and notes).
%
%   Requires: base MATLAB only.
%
%   See also UPDATEDEMOVIEWER, CREATEDEMOVIEWER.

lines = {};
cfg = frame.cfg;
t   = frame.t;

% ---------------------------------------------------------------------
% [DETECT] a confirmed track we have not announced yet, ahead of the ego
% ---------------------------------------------------------------------
tracks = frame.tracks;
fwd = [cos(frame.ego.heading), sin(frame.ego.heading)];
for i = 1:numel(tracks)
    o = tracks(i);
    if any(st.seenTracks == o.id), continue; end
    rel = o.pos - frame.ego.pos;
    ahead = rel * fwd.';
    if ahead <= 0 || hypot(rel(1), rel(2)) > 45, continue; end
    st.seenTracks(end+1) = o.id;
    lines{end+1} = sprintf('%5.1fs [DETECT]   %s ahead, %.0f m', t, prettyClass(o.class), ahead); %#ok<AGROW>
end

% ---------------------------------------------------------------------
% [DETECT] a pothole the pothole tracker has just confirmed
% ---------------------------------------------------------------------
pt = frame.potholeTracks;
plannerPot = getf(plan, 'potholes', []);
for i = 1:numel(pt)
    if ~pt(i).confirmed || any(st.seenPotholes == pt(i).id), continue; end
    st.seenPotholes(end+1) = pt(i).id;
    action = '';
    if ~isempty(plannerPot)
        li = find([plannerPot.id] == pt(i).id, 1);
        if ~isempty(li) && ~strcmp(plannerPot(li).action, 'none')
            action = sprintf(' -> %s', upper(plannerPot(li).action));
        end
    end
    lines{end+1} = sprintf('%5.1fs [DETECT]   %s pothole, %.0f cm%s', ...
                           t, upper(pt(i).severity), 100*pt(i).depth, action); %#ok<AGROW>
end

% ---------------------------------------------------------------------
% [PREDICT] the predicted conflict that is driving the behaviour
% ---------------------------------------------------------------------
% Logged when the CATEGORY changes (none -> occupancy on path -> timed
% conflict), not whenever the number moves, so the log stays readable. The
% number shown is the value at the moment the category was entered.
ttcP = getf(plan, 'ttcPlanned', Inf);
risk = getf(plan, 'risk', 0);
if isfinite(ttcP) && ttcP <= cfg.risk.ttcWarning
    category = 'conflict';
    note = sprintf('Path conflict predicted in %.1f s', ttcP);
elseif risk >= cfg.decision.riskHazard
    category = 'occupancy';
    note = sprintf('Predicted occupancy on path (risk %.2f)', risk);
else
    category = '';
    note = '';
end
if ~isempty(category) && ~strcmp(category, st.lastPredNote)
    lines{end+1} = sprintf('%5.1fs [PREDICT]  %s', t, note); %#ok<AGROW>
end
st.lastPredNote = category;

% ---------------------------------------------------------------------
% [PLAN] what the planner did about it (its own behaviour flags)
% ---------------------------------------------------------------------
if flag(beh, 'blockedAhead')
    note = 'Stopping before blocked passage';
elseif flag(beh, 'yielding')
    note = 'Holding back for predicted conflict';
elseif flag(beh, 'potholeAvoid')
    note = 'Trajectory deformed around pothole';
elseif flag(beh, 'avoiding')
    note = 'Trajectory deformed around hazard';
elseif flag(beh, 'following')
    note = sprintf('Following %s, keeping gap', prettyClass(getf(beh.lead, 'class', 'vehicle')));
elseif flag(beh, 'potholeSlow')
    note = 'Slowing to cross pothole';
elseif flag(beh, 'narrowPassage')
    note = 'Narrow passage, speed capped';
else
    note = '';
end
if ~isempty(note) && ~strcmp(note, st.lastPlanNote)
    lines{end+1} = sprintf('%5.1fs [PLAN]     %s', t, note); %#ok<AGROW>
end
st.lastPlanNote = note;

% ---------------------------------------------------------------------
% [RISK] the planner could not produce a usable trajectory
% ---------------------------------------------------------------------
status = getf(plan, 'status', 'ok');
if ~strcmp(status, 'ok')
    note = sprintf('No usable trajectory (%s)', strrep(status, '_', ' '));
else
    note = '';
end
if ~isempty(note) && ~strcmp(note, st.lastRiskNote)
    lines{end+1} = sprintf('%5.1fs [RISK]     %s', t, note); %#ok<AGROW>
end
st.lastRiskNote = note;

% ---------------------------------------------------------------------
% [DECISION] the decision logic changed state
% ---------------------------------------------------------------------
state = frame.action.state;
if ~strcmp(state, st.lastState)
    lines{end+1} = sprintf('%5.1fs [DECISION] %s', t, judgeWord(state)); %#ok<AGROW>
    st.lastState = state;
end
if frame.action.emergencyBrake && ~st.lastEmergency
    lines{end+1} = sprintf('%5.1fs [DECISION] EMERGENCY BRAKING', t); %#ok<AGROW>
end
st.lastEmergency = frame.action.emergencyBrake;
end

% =========================================================================
function w = judgeWord(state)
%JUDGEWORD The decision-logic state, in one word a judge can read.
switch state
    case 'NORMAL_DRIVING',       w = 'CRUISE';
    case 'HAZARD_ASSESSMENT',    w = 'SLOW';
    case 'PREDICTIVE_AVOIDANCE', w = 'AVOID';
    case 'CONSERVATIVE_DRIVING', w = 'SLOW (low confidence)';
    case 'SAFE_STOP',            w = 'STOP';
    case 'RECOVERY',             w = 'RESUME';
    otherwise,                   w = state;
end
end

function s = prettyClass(c)
switch c
    case 'autorickshaw', s = 'Auto-rickshaw';
    case 'animal',       s = 'Cattle';
    case 'pushcart',     s = 'Pushcart';
    case 'motorcycle',   s = 'Motorcycle';
    case 'pedestrian',   s = 'Pedestrian';
    case 'bicycle',      s = 'Bicycle';
    case 'unknown',      s = 'Unknown object';
    otherwise
        if isempty(c), s = 'Object'; else, s = [upper(c(1)) c(2:end)]; end
end
end

function tf = flag(s, name)
tf = isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && logical(s.(name));
end

function v = getf(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
