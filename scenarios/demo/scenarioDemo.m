function scn = scenarioDemo(res)
%SCENARIODEMO Primary judge demonstration: one road, every IR-PSC behaviour.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION (MATLAB-defined scenario, NOT a
%   RoadRunner scene)
%
%   scn = SCENARIODEMO(res) builds a ~330 m Indian two-way road that takes
%   the vehicle through a scripted sequence of situations, each designed to
%   make one part of the IR-PSC hierarchy visible. Events are triggered by
%   the ego's progress along the road ('TriggerS'), so they happen at the
%   intended place whatever speed the planner chooses.
%
%   The sequence (road station s, metres)
%   -------------------------------------
%     0-110   Marked approach road (centre and edge lines are DRAWN, never
%             used by the planner). Normal driving; an oncoming car on its
%             own (left) side.
%      62     MODERATE pothole near the centre. Road is clear, so the
%             deformation steers around it and returns to the preferred path.
%      80     MINOR pothole narrower than the wheel track: straddled, no
%             reaction needed.
%   110-...   Markings end. A slow BICYCLE ahead in the path while an
%             oncoming BUS occupies the other half: no room to pass, so the
%             vehicle FOLLOWS, then overtakes once the bus has passed.
%   150-200   Village market stretch: road narrows, a PARKED TRUCK and a
%             vegetable CART take up the left side. The drivable corridor
%             visibly shifts right and the path bends around them.
%     206     SEVERE pothole on the right half, just after the truck: steer
%             around it.
%     222     Wide BROKEN PATCH (moderate) across almost the whole road:
%             cannot be avoided, so the vehicle SLOWS and continues.
%     262     COW walks into the road, stands, then leaves: predicted
%             occupancy grows, the vehicle yields / stops in a controlled
%             way and continues when the path clears.
%     292     PEDESTRIAN crosses away from any crossing.
%     320     Goal.
%
%   What must not be claimed: that this is a RoadRunner scene, that actor
%   behaviour is realistic beyond scripted paths, or that the potholes are
%   detected by a trained detector (see DETECTPOTHOLES).
%
%   Inputs:
%       res - grid resolution, m per cell
%
%   Outputs:
%       scn - scenario struct (see BUILDSCENARIO)
%
%   Requires: base MATLAB only.
%
%   See also BUILDSCENARIO, ROADPATH, MAKEACTOR, MAKEPOTHOLE, RUNDEMO.

% ---------------------------------------------------------------------
% Road geometry
% ---------------------------------------------------------------------
sVals = (0:2:336).';
xC = sVals;
yC = 7*sin(sVals/75) + 1.5*sin(sVals/27);
centerline = [xC, yC];

halfWidth = 3.6 + 0.20*sin(sVals/13);          % ~7 m two-way road
market = sVals >= 146 & sVals <= 234;
halfWidth(market) = 3.1 + 0.15*sin(sVals(market)/7);
halfWidth = max(halfWidth, 2.9);

sCl  = pathArcLength(centerline);
thCl = pathHeading(centerline);
at   = @(s, d) frenetToCartesian(centerline, s, d);
hdgAt = @(s) interp1(sCl, unwrap(thCl), s, 'linear');

% ---------------------------------------------------------------------
% Static objects. inGrid = true objects are non-drivable cells the planner
% sees; the rest (houses, trees) stand beyond the road edge and are drawn
% only.
% ---------------------------------------------------------------------
statics = struct('type',{},'center',{},'halfSize',{},'radius',{}, ...
                 'yaw',{},'height',{},'inGrid',{});

% Parked truck on the left of the market stretch.
statics(end+1) = staticBox('truck',  at(178, 2.05), [4.0, 1.22], hdgAt(178), 3.2, true);
% Vegetable cart and a stall canopy just before it.
statics(end+1) = staticBox('cart',   at(160, 2.35), [0.9, 0.6], hdgAt(160), 1.1, true);
statics(end+1) = staticBox('stall',  at(152, 3.15), [1.6, 0.8], hdgAt(152), 2.4, true);
statics(end+1) = staticBox('stall',  at(196, -3.05), [1.5, 0.7], hdgAt(196), 2.4, true);
% Roadside debris (bricks) at the edge.
statics(end+1) = staticDisc('debris', at(128, -3.3), 0.45, 0.4, true);
statics(end+1) = staticDisc('debris', at(240, 3.0), 0.40, 0.4, true);

% Houses and trees beyond the road edge (display only).
for s0 = 12:24:330
    side = 1 - 2*mod(round(s0/24), 2);
    off  = side * (interp1(sVals, halfWidth, s0, 'linear') + 9 + 2*mod(s0, 3));
    statics(end+1) = staticBox('building', at(s0, off), [3.5 + mod(s0,5)*0.4, 3.0], ...
                         hdgAt(s0), 3.5 + mod(s0,4), false); %#ok<AGROW>
end
for s0 = 6:15:330
    side = 1 - 2*mod(round(s0/15), 2);
    statics(end+1) = staticDisc('tree', at(s0, side * (interp1(sVals, halfWidth, s0, 'linear') + 4.5)), 0.9, ...
                          5 + mod(s0, 3), false); %#ok<AGROW>
end

extras = struct('center', {}, 'radius', {}, 'halfSize', {}, 'yaw', {});
for i = 1:numel(statics)
    if statics(i).inGrid
        extras(end+1) = struct('center', statics(i).center, ...
                               'radius', statics(i).radius, ...
                               'halfSize', statics(i).halfSize, ...
                               'yaw', statics(i).yaw); %#ok<AGROW>
    end
end

grid = buildRoadGrid(centerline, halfWidth, res, extras);

% ---------------------------------------------------------------------
% Potholes (drivable surface hazards, NOT in the occupancy grid)
% ---------------------------------------------------------------------
potholes = makePothole(1, at(62, 0.15),  1.4, 1.5, 0.07, 'Yaw', hdgAt(62));    % moderate, too wide to straddle
potholes(end+1) = makePothole(2, at(80, 0.0),   0.50, 0.40, 0.03, 'Yaw', hdgAt(80));  % minor, straddle
potholes(end+1) = makePothole(3, at(206, -0.9), 1.6, 1.4, 0.14, 'Yaw', hdgAt(206));   % severe
potholes(end+1) = makePothole(4, at(222, -0.1), 1.1, 5.2, 0.06, 'Yaw', hdgAt(222));   % wide patch

% ---------------------------------------------------------------------
% Road users. Offsets d are to the LEFT of the road direction; India
% drives on the left, so oncoming traffic uses negative d.
% ---------------------------------------------------------------------
actors = makeActor(1, 'car', roadPath(centerline, 115, 0, -1.9), 9.0, ...
                   'TriggerS', 0);
actors(end+1) = makeActor(2, 'bicycle', roadPath(centerline, 100, 168, 0.9), 3.2, ...
                          'TriggerS', 40, 'StopAtEnd', false);   % turns off into a lane at s=168
% Timed to meet the ego while it is behind the bicycle (~s = 130-145), on
% the wide stretch -- NOT inside the narrow market, where a scripted bus
% that cannot yield would drive into a waiting car.
actors(end+1) = makeActor(3, 'bus', roadPath(centerline, 250, 0, -1.8), 8.0, ...
                          'TriggerS', 40);
actors(end+1) = makeActor(4, 'autorickshaw', roadPath(centerline, 330, 0, -1.7), 6.0, ...
                          'TriggerS', 150);
cowWp = [at(262, -6.0); at(262.5, -2.5); at(263, 0.4); at(263.5, 3.0); at(264, 6.5)];
actors(end+1) = makeActor(5, 'animal', cowWp, 0.9, 'TriggerS', 232, ...
                          'Dwell', [0; 0; 2.5; 0; 0]);
pedWp = [at(292, 5.5); at(292, -5.5)];
actors(end+1) = makeActor(6, 'pedestrian', pedWp, 1.3, 'TriggerS', 270);
actors(end+1) = makeActor(7, 'motorcycle', roadPath(centerline, 0, 336, 0.6), 9.0, ...
                          'TriggerS', 20, 'Follower', true);

% ---------------------------------------------------------------------
scn.name          = 'demo';
scn.profile       = 'demo';
scn.grid          = grid;
scn.centerline    = centerline;
scn.roadHalfWidth = halfWidth;
scn.egoStart      = struct('pos', at(4, 0), 'heading', hdgAt(4), 'speed', 7.0);
scn.goal          = at(320, 0);
scn.goalRadius    = 6.0;
scn.actors        = actors;
scn.potholes      = potholes;
scn.staticObjects = statics;
scn.extras        = extras;
scn.markings      = struct('sFrom', 0, 'sTo', 110);
scn.environment   = 'rural';
scn.duration      = 90.0;
scn.sihScenario   = 'Primary demo: mixed unstructured Indian road';
scn.description   = ['Marked approach that becomes an unmarked village ' ...
                     'road: potholes to avoid, straddle and slow for, a ' ...
                     'bicycle to follow past oncoming traffic, a parked ' ...
                     'truck in a narrow market, a cow and a pedestrian ' ...
                     'crossing.'];
end

% -------------------------------------------------------------------------
function o = staticBox(type, c, halfSize, yaw, height, inGrid)
o = struct('type', type, 'center', c(:).', 'halfSize', halfSize, 'radius', [], ...
           'yaw', yaw, 'height', height, 'inGrid', inGrid);
end

function o = staticDisc(type, c, radius, height, inGrid)
o = struct('type', type, 'center', c(:).', 'halfSize', [], 'radius', radius, ...
           'yaw', 0, 'height', height, 'inGrid', inGrid);
end
