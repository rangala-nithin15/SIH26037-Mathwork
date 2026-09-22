function scn = scenarioUrbanIntersection(res)
%SCENARIOURBANINTERSECTION SIH scenario B: busy unsignalized intersection.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION
%   (NOT a RoadRunner scene. The SIH problem statement requires a detailed
%   RoadRunner urban-intersection scene; this is the MATLAB stand-in.
%   See roadrunner/ROADRUNNER_PLAN.md.)
%
%   scn = SCENARIOURBANINTERSECTION(res) builds a four-way junction with no
%   traffic signal and no marked right of way.
%
%   WHAT THIS SCENARIO IS DESIGNED TO TEST
%   --------------------------------------
%   Prediction under crossing traffic. Nothing here can be solved by
%   following the road: the hazards approach from the side, and the only way
%   to handle them well is to predict where they will be when the ego
%   arrives, rather than where they are now.
%
%   Specific challenges built in:
%     - Cross traffic from BOTH directions, timed to arrive at the junction
%       at roughly the same moment as the ego.
%     - No signal and no priority, so nobody yields. The scripted actors
%       will not stop for the ego under any circumstances.
%     - An auto-rickshaw merging informally from a side road without
%       signalling.
%     - A pedestrian crossing away from any marked crossing point.
%     - Building corners restrict visibility into the junction, so cross
%       traffic is occluded until relatively late -- this is what makes the
%       occlusion model in simulateDetections matter.
%
%   Expected discriminator: the baseline planner treats obstacles as static,
%   so a car that will be in the junction in 1.5 seconds is invisible to it
%   until it is already there. IR-PSC's predicted-occupancy risk sees the
%   conflict developing and can slow or shift early.
%
%   Inputs:
%       res - grid resolution in metres per cell
%
%   Outputs:
%       scn - scenario struct (see buildScenario for the field list)
%
%   Requires: base MATLAB only.
%
%   See also BUILDSCENARIO, PREDICTOBSTACLES, SIMULATEDETECTIONS.

% --- Main road, west to east, running through the junction --------------
xC = (-45:2:55).';
yC = zeros(size(xC));
centerline = [xC, yC];

halfWidth = repmat(3.6, size(xC));
% The junction mouth is wider, as junctions are.
halfWidth(abs(xC) < 9) = 8.5;

% --- Buildings at the corners, which also block visibility ---------------
extras = struct('center', {}, 'radius', {}, 'halfSize', {}, 'yaw', {});
corners = [ -16, -14;  -16, 14;  16, -14;  16, 14 ];
for i = 1:size(corners,1)
    extras(end+1) = struct('center', corners(i,:), 'radius', [], ...
                           'halfSize', [6.0, 5.5], 'yaw', 0); %#ok<AGROW>
end

grid = buildRoadGrid(centerline, halfWidth, res, extras);

% --- Carve the crossing road so cross traffic has somewhere to drive ----
crossCl = [0, -30; 0, 30];
gridCross = buildRoadGrid(crossCl, 3.6, res, []);
grid = mergeFreeSpace(grid, gridCross);
grid = gridDistanceField(grid, 3.0);     % occupancy changed: refresh the field

% --- Actors --------------------------------------------------------------
% Cross traffic southbound, timed to reach the junction with the ego.
actors = makeActor(1, 'car', [0, 28; 0, -30], 8.5, 'StartTime', 0);

% Cross traffic northbound.
actors(end+1) = makeActor(2, 'car', [1.6, -28; 1.6, 30], 7.5, 'StartTime', 1.5);

% Auto-rickshaw merging informally from the side road, no signal.
actors(end+1) = makeActor(3, 'autorickshaw', ...
                          [-2.0, -22; -2.0, -6; 6, -1.5; 40, -1.5], ...
                          6.0, 'StartTime', 2.0);

% Bus proceeding through the junction on the main road, oncoming.
actors(end+1) = makeActor(4, 'bus', [50, -2.0; -40, -2.0], 7.0, 'StartTime', 0);

% Pedestrian crossing away from any marked crossing.
actors(end+1) = makeActor(5, 'pedestrian', [22, -7; 22, 7], 1.3, ...
                          'StartTime', 3.5);

% Motorcycle filtering through, as they do.
actors(end+1) = makeActor(6, 'motorcycle', [-40, 1.8; 50, 1.0], 11.0, ...
                          'StartTime', 1.0);

% --- Assemble ------------------------------------------------------------
scn.name        = 'urban_intersection';
scn.profile     = 'urban';
scn.grid        = grid;
scn.centerline  = centerline;
scn.egoStart    = struct('pos', [-42, 0], 'heading', 0, 'speed', 8.0);
scn.goal        = [50, 0];
scn.goalRadius  = 5.0;
scn.actors      = actors;
scn.extras      = extras;
scn.extraTypes  = repmat({'building'}, 1, numel(extras));
scn.environment = 'urban';
scn.duration    = 40.0;
scn.sihScenario = 'B: Busy unsignalized urban intersection';
scn.description = ['Four-way junction, no signal, no marked right of way. ' ...
                   'Cross traffic from both directions timed to conflict, ' ...
                   'an auto-rickshaw merging informally without signalling, ' ...
                   'an oncoming bus, a filtering motorcycle, and a ' ...
                   'pedestrian crossing away from any marked crossing. ' ...
                   'Corner buildings occlude the junction approaches.'];
end

% =====================================================================
function g = mergeFreeSpace(g, gOther)
%MERGEFREESPACE Mark cells free in g wherever they are free in gOther.
%   The two grids may have different extents, so cells are matched by world
%   coordinates rather than by index.
[cols, rows] = meshgrid(1:g.nCols, 1:g.nRows);
X = g.origin(1) + (cols - 1) * g.resolution;
Y = g.origin(2) + (rows - 1) * g.resolution;

freeElsewhere = ~isOccupiedAt(gOther, [X(:), Y(:)]);
g.occ(reshape(freeElsewhere, size(g.occ))) = false;
end
