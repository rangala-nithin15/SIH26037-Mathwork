function scn = scenarioHighwayMerge(res)
%SCENARIOHIGHWAYMERGE SIH scenario C: highway merge with slow vehicles.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION
%
%   scn = SCENARIOHIGHWAYMERGE(res) builds a higher-speed road on which the
%   ego must deal with slow-moving traffic and a vehicle merging in without
%   signalling.
%
%   WHAT THIS SCENARIO IS DESIGNED TO TEST
%   --------------------------------------
%   Behaviour at speed, where decisions must be made much earlier because
%   the vehicle covers far more ground per second. The corridor lookahead is
%   longer in the 'highway' profile (70 m rather than 40 m) precisely
%   because reacting at 40 m when travelling at 22 m/s leaves under two
%   seconds.
%
%   Specific challenges built in:
%     - A slow truck directly ahead, travelling at roughly a third of the
%       ego's cruising speed. Closing speed is high.
%     - A vehicle merging from an on-ramp WITHOUT signalling, whose path
%       intersects the ego's at a shallow angle. Shallow-angle conflicts are
%       the hardest kind to detect early, because lateral separation shrinks
%       slowly right up until it does not.
%     - A second slow vehicle further ahead, so that overtaking the first
%       does not simply solve the problem.
%     - Wide road, so there IS room to manoeuvre -- this scenario tests
%       whether the planner uses that room in time, not whether it can
%       squeeze through a gap.
%
%   Expected discriminator: time-to-conflict and predicted occupancy give
%   IR-PSC early warning of the shallow-angle merge. A static-obstacle
%   planner sees no conflict until the merging vehicle is already alongside.
%
%   Inputs:
%       res - grid resolution in metres per cell
%
%   Outputs:
%       scn - scenario struct (see buildScenario for the field list)
%
%   Requires: base MATLAB only.
%
%   See also BUILDSCENARIO, TIMETOCONFLICT.

% --- Road: long, wide, gently curving ------------------------------------
xC = (0:4:300).';
yC = 10*sin(xC/150);
centerline = [xC, yC];

halfWidth = repmat(5.5, size(xC));
% The road widens where the on-ramp joins.
halfWidth(xC > 60 & xC < 140) = 7.5;

extras = struct('center', {}, 'radius', {}, 'halfSize', {}, 'yaw', {});
% Crash barrier segments along the outer edge.
for xb = 20:40:280
    j = find(xC >= xb, 1);
    p = centerline(j,:) + (halfWidth(j) + 0.4) * [0, 1];
    extras(end+1) = struct('center', p, 'radius', [], ...
                           'halfSize', [6.0, 0.3], 'yaw', 0); %#ok<AGROW>
end

grid = buildRoadGrid(centerline, halfWidth, res, extras);

% Phase 2: the on-ramp is now part of the drivable grid, so the merging car
% no longer drives across non-drivable space.
mergeStart = centerline(18,:) + [0, -9.0];
mergeMid   = centerline(28,:) + [0, -4.0];
mergeEnd   = centerline(60,:) + [0, -0.5];
rampCl = resamplePath([mergeStart; mergeMid; centerline(34,:) + [0, -2.5]], 40);
rampGrid = buildRoadGrid(rampCl, 2.2, res, []);
[cols, rows] = meshgrid(1:grid.nCols, 1:grid.nRows);
Xg = grid.origin(1) + (cols - 1) * res;
Yg = grid.origin(2) + (rows - 1) * res;
rampFree = ~isOccupiedAt(rampGrid, [Xg(:), Yg(:)]);
grid.occ(reshape(rampFree, size(grid.occ))) = false;
grid = gridDistanceField(grid, 3.0);

% --- Actors --------------------------------------------------------------
% Slow truck directly ahead.
truckPath = roadPath(centerline, 56, 300, -1.5);
actors = makeActor(1, 'truck', truckPath, 8.0, 'StartTime', 0);

% Second slow vehicle further ahead, so overtaking once is not enough.
busPath = roadPath(centerline, 136, 300, 1.5);
actors(end+1) = makeActor(2, 'bus', busPath, 9.5, 'StartTime', 0);

% Merging vehicle: enters from the right at a shallow angle, no signal.
actors(end+1) = makeActor(3, 'car', [resamplePath([mergeStart; mergeMid], 12); ...
                          roadPath(centerline, 112, 236, -0.5)], ...
                          14.0, 'StartTime', 1.0);

% An auto-rickshaw travelling well below the flow speed.
actors(end+1) = makeActor(4, 'autorickshaw', roadPath(centerline, 196, 300, 3.0), ...
                          6.5, 'StartTime', 0);

% --- Assemble ------------------------------------------------------------
scn.name        = 'highway_merge';
scn.profile     = 'highway';
scn.grid        = grid;
scn.centerline  = centerline;
scn.egoStart    = struct('pos', centerline(2,:), 'heading', 0, 'speed', 18.0);
scn.goal        = centerline(end-3,:);
scn.goalRadius  = 8.0;
scn.actors      = actors;
scn.duration    = 45.0;
scn.roadHalfWidth = halfWidth;
scn.extras      = extras;
scn.extraTypes  = repmat({'barrier'}, 1, numel(extras));
scn.markings    = struct('sFrom', 0, 'sTo', 300);
scn.environment = 'highway';
scn.sihScenario = 'C: Highway merge with slow-moving vehicles';
scn.description = ['Wide higher-speed road. A slow truck ahead, a slow bus ' ...
                   'beyond it, a slow auto-rickshaw, and a car merging from ' ...
                   'an on-ramp at a shallow angle without signalling. Tests ' ...
                   'early decision-making at speed, where reaction distance ' ...
                   'dominates.'];
end
