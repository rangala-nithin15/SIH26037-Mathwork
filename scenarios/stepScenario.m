function [actors, groundTruth] = stepScenario(actors, t, dt, egoInfo)
%STEPSCENARIO Advance all scripted actors and emit ground truth.
%
%   COMPONENT STATUS: REAL
%
%   [actors, groundTruth] = STEPSCENARIO(actors, t, dt) moves every active
%   actor along its waypoint path and returns the current true state of all
%   of them in the standard obstacle format.
%   [actors, groundTruth] = STEPSCENARIO(actors, t, dt, egoInfo) also
%   supports the Phase 2 opt-in behaviours of MAKEACTOR ('TriggerS' and
%   'Follower'), which need to know where the ego is.
%
%   An actor becomes active when the simulation time reaches its startTime
%   (and, if it has one, when the ego reaches its trigger station). That
%   mechanism produces the sudden appearances these scenarios need: cattle
%   stepping into the road, a pedestrian leaving the verge. The road user
%   simply does not exist until its moment, and then it does.
%
%   Velocity is taken from the path tangent, so heading and velocity always
%   agree. Perception noise is applied downstream in SIMULATEDETECTIONS --
%   the values here are exact ground truth, which is what the collision
%   referee and the prediction error metric compare against.
%
%   Inputs:
%       actors  - actor struct array from makeActor()
%       t       - current simulation time (s)
%       dt      - time step (s)
%       egoInfo - (optional) struct: .pos .heading .speed .roadS (ego
%                 station along the scenario centreline) .length .width
%                 .centerOffset (body centre ahead of the rear axle, m)
%
%   Outputs:
%       actors      - updated actor struct array (pass back in next step)
%       groundTruth - obstacle struct array of currently active actors, in
%                     makeObstacle() format, with .truthId = actor id
%
%   Example:
%       [actors, gt] = stepScenario(actors, t, 0.05, egoInfo);
%
%   Requires: base MATLAB only.
%
%   See also MAKEACTOR, MAKEOBSTACLE, SIMULATEDETECTIONS.

if nargin < 4, egoInfo = []; end

groundTruth = struct('id',{},'class',{},'pos',{},'vel',{},'accel',{}, ...
                     'heading',{},'speed',{},'length',{},'width',{}, ...
                     'confidence',{},'age',{},'posCov',{},'vulnerable',{}, ...
                     'agility',{},'truthId',{});

[classNames, classInfo] = objectClasses();

for i = 1:numel(actors)
    a = actors(i);

    % --- Activation ---------------------------------------------------
    if ~a.active
        timeOk = t >= a.startTime;
        trigOk = isnan(a.triggerS) || ...
                 (~isempty(egoInfo) && isfield(egoInfo, 'roadS') && egoInfo.roadS >= a.triggerS);
        if timeOk && trigOk && ~a.finished
            a.active = true;
            if a.nextWp == 1 && a.dwell(1) > 0
                a.dwellUntil = t + a.dwell(1);
                a.nextWp = 2;
            end
        else
            actors(i) = a;
            continue;                    % not yet in the scene
        end
    end

    % --- Longitudinal speed: nominal, dwelling, or keeping a gap ---------
    vWanted = a.speed;
    if t < a.dwellUntil
        vWanted = 0;
    end
    if a.follower && ~isempty(egoInfo)
        vWanted = min(vWanted, followerSpeed(a, egoInfo, classNames, classInfo));
    end
    if a.follower
        % Followers change speed with bounded acceleration.
        dv = max(min(vWanted - a.curSpeed, 3.0 * dt), -6.0 * dt);
        a.curSpeed = max(a.curSpeed + dv, 0);
    else
        a.curSpeed = vWanted;
    end

    % --- Advance along the waypoint path ------------------------------
    if ~a.finished
        sNew = a.s + a.curSpeed * dt;
        % Arriving at a waypoint with a dwell time: stop exactly there.
        if a.nextWp <= numel(a.wpS) && sNew >= a.wpS(a.nextWp)
            if a.dwell(a.nextWp) > 0
                sNew = a.wpS(a.nextWp);
                a.dwellUntil = t + a.dwell(a.nextWp);
            end
            a.nextWp = a.nextWp + 1;
        end
        a.s = sNew;

        if a.s >= a.pathLength
            if a.loop
                a.s = mod(a.s, max(a.pathLength, eps));
                a.nextWp = 2;
            elseif a.stopAtEnd
                a.s = a.pathLength;
                a.finished = true;
            else
                a.active = false;        % leaves the scene entirely
                a.finished = true;
                actors(i) = a;
                continue;
            end
        end
    end

    % --- Pose from the path -------------------------------------------
    sU = a.wpS;
    wU = a.waypoints;
    keep = [true; diff(sU) > 1e-9];
    sU = sU(keep);  wU = wU(keep,:);
    if numel(sU) < 2
        pos = a.waypoints(1,:);
        hdg = a.heading;
    else
        sq  = min(max(a.s, 0), sU(end));
        pos = [interp1(sU, wU(:,1), sq, 'linear'), ...
               interp1(sU, wU(:,2), sq, 'linear')];
        ds  = min(0.5, sU(end) / 20);
        sA  = min(max(sq - ds, 0), sU(end));
        sB  = min(max(sq + ds, 0), sU(end));
        pA  = [interp1(sU, wU(:,1), sA, 'linear'), interp1(sU, wU(:,2), sA, 'linear')];
        pB  = [interp1(sU, wU(:,1), sB, 'linear'), interp1(sU, wU(:,2), sB, 'linear')];
        if hypot(pB(1)-pA(1), pB(2)-pA(2)) > 1e-6
            hdg = atan2(pB(2)-pA(2), pB(1)-pA(1));
        else
            hdg = a.heading;
        end
    end

    if a.finished || t < a.dwellUntil
        spd = 0;
    else
        spd = a.curSpeed;
    end

    a.pos     = pos;
    a.heading = hdg;
    a.vel     = spd * [cos(hdg), sin(hdg)];
    actors(i) = a;

    groundTruth(end+1) = makeObstacle(a.id, a.class, a.pos, a.vel, ...
                                      'Heading', a.heading, ...
                                      'Confidence', 1.0, ...
                                      'Age', 100, ...
                                      'TruthId', a.id); %#ok<AGROW>
end
end

% -------------------------------------------------------------------------
function v = followerSpeed(a, ego, classNames, classInfo)
% Gap keeping to the ego for an actor travelling in the same direction
% behind it. Returns Inf when the ego is not ahead of this actor.
v = Inf;
ci = find(strcmp(classNames, a.class), 1);
if isempty(ci), return; end
aLen = classInfo(ci).length;
aWid = classInfo(ci).width;

egoC = ego.pos(:).' + ego.centerOffset * [cos(ego.heading), sin(ego.heading)];
fwd  = [cos(a.heading), sin(a.heading)];
rel  = egoC - a.pos;
along = rel * fwd.';
lat   = abs(rel * [-fwd(2); fwd(1)]);
sameDir = cos(ego.heading - a.heading) > 0.5;
if ~sameDir || along <= 0 || along > 40 || lat > (ego.width + aWid)/2 + 0.5
    return;
end
gap = along - ego.length/2 - aLen/2;
v = max(0, min(max(ego.speed, 0) + 0.5 * (gap - 3.0), (gap - 2.0) / 0.8));
end
