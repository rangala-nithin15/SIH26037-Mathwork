function vc = vizConfig(mode, overrides)
%VIZCONFIG Single place to configure the IR-PSC 3D visualisation.
%
%   COMPONENT STATUS: REAL (visualisation settings only).
%
%   This file controls ONLY how the simulation is drawn. Nothing here can
%   change the planner, the prediction, the risk model, the decision logic,
%   the controller or the vehicle dynamics: the renderer consumes finished
%   simulation state and never feeds anything back. Planner settings live in
%   config/irpscConfig.m and are untouched by this file.
%
%   vc = VIZCONFIG()              settings with the default visual mode
%   vc = VIZCONFIG(mode)          'full' | 'planning' | 'sensing' | 'minimal'
%   vc = VIZCONFIG(mode, over)    the same, with a struct of overrides applied
%                                 (nested structs are merged field by field)
%
%   Example:
%       vc = vizConfig('planning', struct('displayFps', 60));
%       runDemo(struct('viz', vc))
%
%   ---------------------------------------------------------------------
%   WHERE TO CHANGE THINGS
%   ---------------------------------------------------------------------
%     display FPS     -> vc.displayFps         (section "Timing")
%     visual mode     -> vc.visualMode         (section "Timing", or arg 1)
%     camera mode     -> vc.camera             (section "Timing")
%     colours         -> section "Semantic palette"
%     transparency    -> section "Transparency"
%     layer rates     -> section "Layer update rates"
%     what is drawn   -> section "Layer visibility" (HUD, prediction, risk,
%                        corridor, trail, performance HUD, ...)
%
%   Requires: base MATLAB only.
%
%   See also CREATEDEMOVIEWER, UPDATEDEMOVIEWER, RUNDEMO, REPLAYDEMO.

if nargin < 1 || isempty(mode), mode = 'full'; end
if nargin < 2, overrides = struct(); end

% =====================================================================
% Timing, mode and camera
% =====================================================================
vc.displayFps  = 30;        % target frames per second for REPLAYDEMO
vc.fpsOptions  = [15 20 30 45 60];   % supported targets (15/20/30/45/60)
vc.visualMode  = mode;      % 'full' | 'planning' | 'sensing' | 'minimal'
vc.camera      = 'chase';   % 'chase' | 'overview' | 'top'

% Visual interpolation: draw the ego vehicle and camera between simulation
% states so motion looks continuous. This moves only the PICTURE between two
% states the simulation already produced; it never invents simulation state,
% and the logged trajectory, poses and decisions are untouched.
vc.interpolate    = true;
vc.maxTweenFrames = 4;      % live runs: extra interpolated frames per sim step
vc.cameraLag      = 0.18;   % 0 = camera snaps to the ego, 1 = never catches up
                            % (applied per second of display time, not per frame)

% =====================================================================
% Layer visibility (base values; the visual mode below overrides some)
% =====================================================================
vc.show.hud          = true;   % right-hand text panel
vc.show.perfHud      = true;   % target/actual FPS, frame time  (easy on/off)
vc.show.events       = true;   % [DETECT]/[PREDICT]/[PLAN]/[DECISION] log
vc.show.corridor     = true;   % green drivable corridor
vc.show.corridorEdge = true;
vc.show.trajectory   = true;   % cyan planned trajectory
vc.show.preferred    = true;   % neutral preferred path
vc.show.trail        = true;   % fading trail behind the ego
vc.show.risk         = true;   % risk cells (the DP's own table)
vc.show.prediction   = true;   % predicted occupancy ellipses
vc.show.predPaths    = true;   % predicted centre paths
vc.show.tracks       = true;   % tracker boxes and labels
vc.show.detections   = true;   % raw camera/LiDAR/radar returns
vc.show.potholes     = true;   % pothole rings and labels
vc.show.potholeCost  = true;   % pothole cost cells used by the DP
vc.show.minimap      = true;
vc.show.timeseries   = true;
vc.show.rejected     = true;   % rejected candidate trajectory
vc.show.markings     = true;   % painted lines (display only; planner ignores)

vc.trailSeconds  = 7;          % length of the ego trail
vc.trailMaxPoints = 90;        % points drawn in that trail (thinned above this)
vc.cellStride    = 2;          % merge N lateral offsets per displayed risk cell
                               % (display only: the DP still uses every offset)
vc.eventLines    = 5;          % visible event-log lines
vc.potholeLines  = 4;          % visible pothole rows in the HUD

% =====================================================================
% Layer update rates (in rendered frames; 1 = every frame)
% ---------------------------------------------------------------------
% The ego, its trail, the trajectory and the camera always update every
% frame. Slower-changing and expensive layers may update less often: in
% GNU Octave each graphics property write costs about a millisecond, and
% these layers are the ones with many handles behind them.
% =====================================================================
vc.rate.risk        = 2;
vc.rate.prediction  = 2;
vc.rate.potholeCost = 3;
vc.rate.corridor    = 1;
vc.rate.tracks      = 2;
vc.rate.detections  = 2;
vc.rate.potholes    = 2;
vc.rate.hud         = 2;
vc.rate.minimap     = 3;
vc.rate.timeseries  = 6;

% =====================================================================
% Semantic palette
% ---------------------------------------------------------------------
% Every colour means one thing and keeps that meaning everywhere:
%   teal/green  = space the vehicle may use      (drivable corridor)
%   cyan        = what the vehicle intends to do (planned trajectory)
%   amber       = caution, waiting, following
%   red         = stop / high risk
%   orange-red  = predicted occupancy of others
%   neutrals    = the world (road, buildings, parked things)
% =====================================================================
c = struct();
% -- interface
c.bg            = [0.055 0.066 0.086];   % figure background
c.panel         = [0.086 0.102 0.130];   % HUD panel
c.panelEdge     = [0.180 0.216 0.267];
c.sky           = [0.118 0.149 0.196];   % horizon behind the scene
c.text          = [0.902 0.933 0.965];
c.textMuted     = [0.529 0.588 0.659];
c.accent        = [0.133 0.878 1.000];   % cyan: the system's own voice
% -- world (muted, so data layers stand out)
c.terrain       = [0.133 0.169 0.141];
c.road          = [0.157 0.169 0.192];   % dark neutral asphalt
c.roadEdge      = [0.235 0.251 0.282];
c.shoulder      = [0.239 0.216 0.176];
c.marking       = [0.749 0.769 0.741];
c.building      = [0.361 0.333 0.298];
c.buildingRoof  = [0.286 0.204 0.180];
c.tree          = [0.176 0.310 0.196];
c.treeTrunk     = [0.243 0.184 0.133];
c.staticObj     = [0.400 0.376 0.333];   % parked/roadside things: muted
c.potholeFloor  = [0.043 0.047 0.055];
c.potholeRim    = [0.110 0.118 0.129];
% -- ego
c.ego           = [0.886 0.925 0.965];
c.egoAccent     = [0.133 0.878 1.000];
c.egoGlass      = [0.114 0.204 0.278];
% -- road users (distinct, same family, never neon)
c.actor.car          = [0.776 0.353 0.294];
c.actor.bus          = [0.839 0.573 0.196];
c.actor.truck        = [0.400 0.502 0.412];
c.actor.autorickshaw = [0.267 0.643 0.541];
c.actor.motorcycle   = [0.651 0.447 0.780];
c.actor.bicycle      = [0.408 0.573 0.812];
c.actor.pedestrian   = [0.886 0.749 0.365];
c.actor.animal       = [0.784 0.706 0.612];
c.actor.pushcart     = [0.596 0.545 0.451];
c.actor.unknown      = [0.502 0.533 0.573];
% -- data layers
c.corridor      = [0.078 0.769 0.588];   % drivable space
c.corridorEdge  = [0.114 0.855 0.655];
c.corridorBad   = [0.851 0.243 0.208];
c.traj          = [0.133 0.878 1.000];   % planned trajectory, normal cruise
c.trajCaution   = [0.976 0.639 0.149];   % following / yielding / blocked
c.trajStop      = [0.937 0.267 0.220];   % safe stop
c.preferred     = [0.600 0.678 0.749];   % preferred path: neutral, thin
c.trail         = [0.443 0.792 0.886];
c.rejected      = [0.851 0.325 0.310];
c.predOcc       = [0.976 0.451 0.231];   % predicted occupancy
c.predPath      = [0.933 0.584 0.388];
c.riskLow       = [0.898 0.827 0.302];   % yellow-green
c.riskMed       = [0.960 0.580 0.180];   % orange
c.riskHigh      = [0.918 0.235 0.196];   % red
c.potholeCost   = [0.643 0.451 0.839];
c.potholeMinor  = [0.898 0.827 0.302];
c.potholeMod    = [0.960 0.580 0.180];
c.potholeSevere = [0.918 0.235 0.196];
c.track         = [0.949 0.878 0.396];
c.detCam        = [0.976 0.878 0.435];
c.detLidar      = [0.416 0.851 0.616];
c.detRadar      = [0.769 0.514 0.878];
c.stopWall      = [0.918 0.235 0.196];
% -- state badges (decision logic)
c.stateNormal   = [0.133 0.545 0.353];
c.stateHazard   = [0.761 0.573 0.114];
c.stateAvoid    = [0.859 0.435 0.102];
c.stateConserv  = [0.180 0.396 0.702];
c.stateStop     = [0.824 0.176 0.153];
c.stateRecover  = [0.114 0.510 0.510];
vc.color = c;

% =====================================================================
% Transparency
% =====================================================================
vc.alpha.corridor    = 0.20;
vc.alpha.riskCell    = 0.50;
vc.alpha.predOcc     = 0.20;
vc.alpha.trajRibbon  = 0.80;
vc.alpha.potholeCost = 0.30;
vc.alpha.stopWall    = 0.40;
vc.alpha.trail       = 0.60;

% =====================================================================
% Visual modes
% =====================================================================
switch lower(vc.visualMode)
    case 'full'
        % everything above stays on
    case 'planning'
        % the planner's reasoning: space, path, hazard. Sensor plumbing off.
        vc.show.detections = false;
        vc.show.tracks     = false;
        vc.show.minimap    = false;
    case 'sensing'
        % what the vehicle perceives, before the planner acts on it
        vc.show.risk        = false;
        vc.show.potholeCost = false;
        vc.show.corridor    = false;
        vc.show.preferred   = false;
        vc.show.rejected    = false;
    case 'minimal'
        % road, ego, road users, trajectory: nothing else
        vc.show.risk        = false;
        vc.show.potholeCost = false;
        vc.show.prediction  = false;
        vc.show.predPaths   = false;
        vc.show.detections  = false;
        vc.show.tracks      = false;
        vc.show.rejected    = false;
        vc.show.preferred   = false;
        vc.show.minimap     = false;
        vc.show.timeseries  = false;
        vc.show.corridor    = false;
    otherwise
        error('vizConfig:unknownMode', ...
              'Unknown visual mode "%s". Use full, planning, sensing or minimal.', vc.visualMode);
end

vc = mergeStruct(vc, overrides);
end

% =========================================================================
function a = mergeStruct(a, b)
%MERGESTRUCT Recursively copy the fields of b onto a.
if ~isstruct(b), return; end
f = fieldnames(b);
for i = 1:numel(f)
    if isfield(a, f{i}) && isstruct(a.(f{i})) && isstruct(b.(f{i}))
        a.(f{i}) = mergeStruct(a.(f{i}), b.(f{i}));
    else
        a.(f{i}) = b.(f{i});
    end
end
end
