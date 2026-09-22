function scn = scenarioMarket(res)
%SCENARIOMARKET SIH scenario D: dense market area with mixed traffic.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION
%
%   scn = SCENARIOMARKET(res) builds a congested market street where the
%   drivable space is narrow, irregular and crowded with vulnerable road
%   users moving unpredictably.
%
%   WHAT THIS SCENARIO IS DESIGNED TO TEST
%   --------------------------------------
%   Everything at once, at low speed. This is the hardest scenario in the
%   set, and it is the one where the confidence-aware behaviour should
%   visibly earn its place: with many low-confidence tracks and uncertain
%   predictions, the system should slow down of its own accord rather than
%   pressing on and relying on luck.
%
%   Specific challenges built in:
%     - Drivable width down to about 3.0 m in places, barely wider than the
%       vehicle plus its margins. The corridor is genuinely tight.
%     - Stalls and parked carts encroaching from both sides at irregular
%       intervals, so the usable centre wanders left and right constantly.
%     - Pedestrians crossing at unpredictable moments and at no marked
%       crossing point, activated at staggered times.
%     - A pushcart moving slowly along the road, effectively a moving
%       roadblock.
%     - A stationary bicycle partially obstructing the path.
%     - A motorcycle weaving through, high agility, hard to predict.
%
%   Expected discriminator: the fixed-candidate baseline has five preset
%   offsets. In a corridor whose usable centre shifts every few metres, the
%   right offset is almost never one of the five, so it should either stop
%   frequently (high emergency-stop count) or fail to find a candidate at
%   all (no_feasible_candidate failures). IR-PSC can follow the wandering
%   centre continuously.
%
%   Inputs:
%       res - grid resolution in metres per cell
%
%   Outputs:
%       scn - scenario struct (see buildScenario for the field list)
%
%   Requires: base MATLAB only.
%
%   See also BUILDSCENARIO, COMPUTECONFIDENCE, DECISIONLOGIC.

% --- Narrow, winding market street ---------------------------------------
sVals = (0:1.5:80).';
xC    = sVals;
yC    = 2.2*sin(sVals/13) + 0.5*sin(sVals/4.5);
centerline = [xC, yC];

% --- Very narrow and highly irregular width ------------------------------
halfWidth = 2.1 + 0.55*sin(sVals/9) + 0.25*sin(sVals/3.1);
halfWidth = max(halfWidth, 1.55);

% --- Stalls and parked carts encroaching from both sides ----------------
extras = struct('center', {}, 'radius', {}, 'halfSize', {}, 'yaw', {});
stallAt = 6:5:76;
for k = 1:numel(stallAt)
    j = find(sVals >= stallAt(k), 1);
    if isempty(j), continue; end
    side = (-1)^k;
    p = centerline(j,:) + side * (halfWidth(j) - 0.15) * [0, 1];
    extras(end+1) = struct('center', p, 'radius', [], ...
                           'halfSize', [1.1, 0.7], 'yaw', 0.1*side); %#ok<AGROW>
end

% Stationary bicycle leaning into the road.
jb = find(sVals >= 40, 1);
% Phase 2: moved from 0.9 m to 1.3 m off the centreline. Together with the
% neighbouring stall it previously left a 1.9 m gap, narrower than the car
% itself, so the market could never be completed.
extras(end+1) = struct('center', centerline(jb,:) + [0, 1.3], 'radius', 0.45, ...
                       'halfSize', [], 'yaw', []);

grid = buildRoadGrid(centerline, halfWidth, res, extras);

% --- Actors --------------------------------------------------------------
% Pushcart moving slowly along the street: a moving roadblock.
actors = makeActor(1, 'pushcart', roadPath(centerline, 13.5, 80, 0.4), ...
                   1.4, 'StartTime', 0);

% Pedestrians crossing at staggered, unpredictable moments.
crossAt   = [14, 26, 38, 52, 64];
startTime = [2.0, 5.5, 9.0, 13.5, 18.0];
for k = 1:numel(crossAt)
    j = find(sVals >= crossAt(k), 1);
    if isempty(j), continue; end
    side = (-1)^k;
    p0 = centerline(j,:) + side * 3.0 * [0, 1];
    p1 = centerline(j,:) - side * 3.0 * [0, 1];
    actors(end+1) = makeActor(10+k, 'pedestrian', [p0; p1], ...
                              0.9 + 0.4*mod(k,2), ...
                              'StartTime', startTime(k)); %#ok<AGROW>
end

% Motorcycle weaving through: high agility, hard to predict.
sW = (7.5:1.5:80).';
weave = frenetToCartesian(centerline, sW, 0.45 * sin((sW - 7.5) / 21 * pi));
actors(end+1) = makeActor(2, 'motorcycle', weave, 5.5, 'StartTime', 1.0);

% Oncoming auto-rickshaw: the street is barely wide enough for both.
actors(end+1) = makeActor(3, 'autorickshaw', roadPath(centerline, 80, 0, -0.25), ...
                          3.2, 'StartTime', 4.0);

% A cow standing at the roadside, occasionally a hazard.
jc = find(sVals >= 58, 1);
actors(end+1) = makeActor(4, 'animal', ...
                          [centerline(jc,:) + [0 2.0]; centerline(jc,:) + [0 0.2]], ...
                          0.5, 'StartTime', 12.0);

% --- Assemble ------------------------------------------------------------
scn.name        = 'market';
scn.profile     = 'market';
scn.grid        = grid;
scn.centerline  = centerline;
scn.egoStart    = struct('pos', centerline(2,:), ...
                         'heading', atan2(centerline(3,2)-centerline(1,2), ...
                                          centerline(3,1)-centerline(1,1)), ...
                         'speed', 3.0);
scn.goal        = centerline(end-2,:);
scn.goalRadius  = 4.0;
scn.actors      = actors;
scn.roadHalfWidth = halfWidth;
scn.extras      = extras;
scn.extraTypes  = [repmat({'stall'}, 1, numel(extras) - 1), {'pole'}];
scn.environment = 'urban';
scn.duration    = 60.0;
scn.sihScenario = 'D: Dense market area with mixed traffic';
scn.description = ['Narrow winding market street, usable width down to ' ...
                   'about 3.0 m, with stalls encroaching irregularly from ' ...
                   'both sides. Five pedestrians crossing at unpredictable ' ...
                   'times and unmarked locations, a slow pushcart, a weaving ' ...
                   'motorcycle, an oncoming auto-rickshaw and a cow at the ' ...
                   'roadside. The hardest scenario in the set.'];
end
