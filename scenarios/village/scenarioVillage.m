function scn = scenarioVillage(res)
%SCENARIOVILLAGE SIH scenario A: unmarked village road.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION
%   (NOT a RoadRunner scene. The SIH problem statement requires a detailed
%   RoadRunner village-road scene; this is the MATLAB stand-in that runs
%   today. See roadrunner/ROADRUNNER_PLAN.md.)
%
%   scn = SCENARIOVILLAGE(res) builds an unmarked rural road.
%
%   WHAT THIS SCENARIO IS DESIGNED TO TEST
%   --------------------------------------
%   This is the purest test of the central IR-PSC claim. There are no lane
%   markings in this scene at all -- not faded ones, none. A planner that
%   depends on lane detection has nothing whatsoever to work with here. A
%   corridor planner is unaffected, because it never asks the question.
%
%   Specific challenges built in:
%     - Width varies from 5.6 m down to 3.8 m, irregularly. The narrowest
%       point is deliberately placed near the parked truck, so the usable
%       gap there is tight but passable.
%     - A parked truck occupies part of the road, as they do.
%     - An oncoming motorcycle shares the road with no lane discipline.
%     - Road-edge debris makes the true boundary irregular rather than a
%       clean line, so boundary rays return uneven distances.
%     - The road curves, so the corridor must bend rather than run straight.
%
%   Expected discriminator: the fixed-candidate baseline must choose one of
%   five preset offsets, none of which is likely to line up with the gap
%   beside the parked truck on a road of varying width. IR-PSC can select
%   any offset continuously.
%
%   Inputs:
%       res - grid resolution in metres per cell
%
%   Outputs:
%       scn - scenario struct (see buildScenario for the field list)
%
%   Requires: base MATLAB only.
%
%   See also BUILDSCENARIO, BUILDROADGRID, MAKEACTOR.

% --- Road geometry: a gently curving rural road -------------------------
sVals = (0:2:120).';
xC    = sVals;
yC    = 6*sin(sVals/40) + 0.8*sin(sVals/11);   % gentle curve plus wander
centerline = [xC, yC];

% --- Irregular width: this is what makes it a village road --------------
K = size(centerline,1);
halfWidth = 2.8 + 0.45*sin(sVals/17) + 0.30*sin(sVals/6.5);
halfWidth(sVals > 50 & sVals < 66) = 1.9;    % pinch point near the truck
halfWidth = max(halfWidth, 1.75);

% --- Static obstructions ------------------------------------------------
extras = struct('center', {}, 'radius', {}, 'halfSize', {}, 'yaw', {});

% Parked truck, partly in the road at the pinch point.
idx = find(sVals >= 56, 1);
% Phase 2: the truck stood 1.5 m left of the centreline, leaving 2.3 m of
% free width -- less than the 1.68 m car plus the project's own 0.25 m
% minimum clearance on each side, discretised on a 0.2 m grid. The scenario
% was therefore impassable by construction. It now leaves ~2.8 m: still a
% NARROW squeeze the vehicle must take slowly, but physically passable.
truckPos = centerline(idx,:) + [0, 2.1];
extras(end+1) = struct('center', truckPos, 'radius', [], ...
                       'halfSize', [4.0, 1.2], 'yaw', 0.05);

% Road-edge debris, so the boundary is irregular rather than a clean line.
debrisAt = [22, 35, 78, 95];
for k = 1:numel(debrisAt)
    j = find(sVals >= debrisAt(k), 1);
    side = (-1)^k;
    p = centerline(j,:) + side * (halfWidth(j) - 0.3) * [0, 1];
    extras(end+1) = struct('center', p, 'radius', 0.55, ...
                           'halfSize', [], 'yaw', []); %#ok<AGROW>
end

grid = buildRoadGrid(centerline, halfWidth, res, extras);

% --- Actors --------------------------------------------------------------
% Phase 2: actors follow the curved road (ROADPATH). The original straight
% two-point paths left the road by up to 5 m.
actors = makeActor(1, 'motorcycle', roadPath(centerline, 120, 0, -1.0), ...
                   7.0, 'StartTime', 0);

% A bicycle travelling the same way as the ego, slowly.
actors(end+1) = makeActor(2, 'bicycle', roadPath(centerline, 22, 120, 0.4), ...
                          3.5, 'StartTime', 0, 'StopAtEnd', false);

% A pedestrian walking along the edge who steps further in partway through.
sPed = linspace(58, 88, 16).';
dPed = linspace(2.2, 0.6, 16).';
pedWp = frenetToCartesian(centerline, sPed, dPed);
actors(end+1) = makeActor(3, 'pedestrian', pedWp, 1.1, 'StartTime', 3.0);

% --- Assemble ------------------------------------------------------------
scn.name        = 'village';
scn.profile     = 'village';
scn.grid        = grid;
scn.centerline  = centerline;
scn.egoStart    = struct('pos', centerline(2,:), ...
                         'heading', atan2(centerline(3,2)-centerline(1,2), ...
                                          centerline(3,1)-centerline(1,1)), ...
                         'speed', 6.0);
scn.goal        = centerline(end-2,:);
scn.goalRadius  = 5.0;
scn.actors      = actors;
scn.duration    = 45.0;
scn.roadHalfWidth = halfWidth;
scn.extras      = extras;
scn.extraTypes  = [{'truck'}, repmat({'debris'}, 1, numel(extras) - 1)];
scn.markings    = struct('sFrom', 0, 'sTo', 0);        % none: unmarked road
% Village roads have potholes; the planner must see them via perception.
scn.potholes    = makePothole(1, frenetToCartesian(centerline, 30, -0.4), 1.2, 1.4, 0.07);
scn.potholes(end+1) = makePothole(2, frenetToCartesian(centerline, 90, 0.5), 0.6, 0.5, 0.03);
scn.sihScenario = 'A: Unmarked village road';
scn.description = ['Unmarked rural road, irregular width 3.8-5.6 m, ' ...
                   'curving, with a parked truck at a pinch point, ' ...
                   'road-edge debris, an oncoming motorcycle, a slow ' ...
                   'bicycle and a pedestrian moving into the road. ' ...
                   'No lane markings exist anywhere in this scene.'];
end
