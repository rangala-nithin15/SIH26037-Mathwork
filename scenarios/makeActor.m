function a = makeActor(id, class, waypoints, speed, varargin)
%MAKEACTOR Define a scripted road user for a scenario.
%
%   COMPONENT STATUS: REAL
%
%   a = MAKEACTOR(id, class, waypoints, speed, ...) defines one road user
%   that moves along a waypoint path at a given speed.
%
%   Actors are SCRIPTED, not intelligent. They follow their waypoints
%   regardless of what the ego vehicle does. That is deliberate for
%   evaluation: a reactive actor that politely yields would quietly make the
%   ego planner look better than it is, and would make runs irreproducible.
%   Scripted actors give every planner an identical, repeatable challenge,
%   which is what the baseline comparison requires.
%
%   The consequence, stated plainly: these scenarios cannot demonstrate
%   negotiation or mutual yielding, because the other road users cannot
%   negotiate. Interactive actor behaviour is future work, and is where
%   RoadRunner Scenario would be used on the destination machine.
%
%   Phase 2 additions (all opt-in; defaults reproduce the original actor)
%   ---------------------------------------------------------------------
%     - Road-following paths: build waypoints with ROADPATH so vehicles stay
%       on a curved road. The original straight two-point paths cut across
%       bends and left the road by up to 5 m.
%     - 'Dwell': a pause at chosen waypoints (a cow that walks into the
%       road, stands, then moves on).
%     - 'TriggerS': activate when the ego reaches a road station, instead of
%       at a fixed time. The event then happens where it is meant to happen
%       whatever speed the planner chose, which keeps different planners'
%       challenges comparable.
%     - 'Follower': a vehicle BEHIND the ego in the same direction keeps a
%       gap instead of driving through the ego. This is the ONLY reaction
%       modelled. Without it the cattle scenario's scripted motorcycle
%       rear-ended the ego before the cattle even appeared, and the metric
%       counted that as the planner's collision. Followers do not yield,
%       negotiate or react to anything else.
%
%   Inputs:
%       id        - numeric scalar, unique actor identifier
%       class     - char, a class name from objectClasses()
%       waypoints - Kx2 path the actor follows, K >= 2
%       speed     - scalar travel speed (m/s)
%
%   Name-value pairs:
%       'StartTime' - s, when the actor becomes active. Default 0.
%       'TriggerS'  - m, activate when the ego's road station reaches this
%                     value (and StartTime has passed). Default NaN (off).
%       'StopAtEnd' - logical, halt at the last waypoint rather than
%                     vanishing. Default true.
%       'Loop'      - logical, restart at the first waypoint. Default false.
%       'Dwell'     - Kx1 seconds to wait on arriving at each waypoint.
%                     Default zeros.
%       'Follower'  - logical, keep a gap to the ego when behind it.
%                     Default false.
%
%   Outputs:
%       a - actor struct with the fields above plus runtime state
%           (.pos, .vel, .heading, .s, .active, .finished, .curSpeed,
%            .dwellUntil, .nextWp)
%
%   Example:
%       wp = roadPath(scn.centerline, 20, 120, 1.2);
%       a  = makeActor(3, 'bicycle', wp, 3.5, 'TriggerS', 5);
%
%   Requires: base MATLAB only.
%
%   See also STEPSCENARIO, ROADPATH, OBJECTCLASSES, MAKEOBSTACLE.

validateattributes(waypoints, {'numeric'}, {'2d','ncols',2,'finite','real'}, ...
                   mfilename, 'waypoints');
if size(waypoints,1) < 2
    error('makeActor:tooFewWaypoints', 'Need at least 2 waypoints.');
end
K = size(waypoints,1);

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'StartTime', 0,     @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'TriggerS',  NaN,   @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'StopAtEnd', true,  @(x) islogical(x) || isnumeric(x));
addParameter(p, 'Loop',      false, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'Dwell',     zeros(K,1), @(x) isnumeric(x) && numel(x) == K);
addParameter(p, 'Follower',  false, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});

a.id        = id;
a.class     = char(class);
a.waypoints = waypoints;
a.speed     = speed;
a.startTime = p.Results.StartTime;
a.triggerS  = p.Results.TriggerS;
a.stopAtEnd = logical(p.Results.StopAtEnd);
a.loop      = logical(p.Results.Loop);
a.dwell     = p.Results.Dwell(:);
a.follower  = logical(p.Results.Follower);

% Runtime state
a.s        = 0;
a.pos      = waypoints(1,:);
a.heading  = atan2(waypoints(2,2)-waypoints(1,2), waypoints(2,1)-waypoints(1,1));
a.vel      = speed * [cos(a.heading), sin(a.heading)];
a.active   = (a.startTime <= 0) && isnan(a.triggerS);
a.finished = false;
a.curSpeed = speed;
a.dwellUntil = -Inf;
a.nextWp   = 2;

sArc = pathArcLength(waypoints);
a.wpS        = sArc;
a.pathLength = sArc(end);

% A dwell at the first waypoint starts as soon as the actor activates.
if a.dwell(1) > 0
    a.nextWp = 1;
end
end
