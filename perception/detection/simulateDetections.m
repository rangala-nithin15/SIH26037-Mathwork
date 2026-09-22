function dets = simulateDetections(groundTruth, ego, sensor, rngStream)
%SIMULATEDETECTIONS Geometric sensor model producing imperfect detections.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION
%
%   dets = SIMULATEDETECTIONS(groundTruth, ego, sensor, rngStream) turns
%   perfect scenario ground truth into the kind of imperfect measurements a
%   real sensor would produce.
%
%   ============================================================
%   WHAT THIS IS, STATED PLAINLY
%   ============================================================
%   This is NOT object detection. No image is processed, no point cloud is
%   segmented, no neural network runs. Ground-truth object positions are
%   taken and corrupted with geometry and noise.
%
%   That is a legitimate and common way to study PLANNING under imperfect
%   perception, and it is what this project studies. It means:
%     - no result from this file may be described as detector performance,
%     - no accuracy, precision or recall figure may be quoted from it,
%     - the planner is nonetheless exercised against realistic failures.
%
%   When Computer Vision, Lidar and Deep Learning Toolboxes are available on
%   the destination machine, this function can be replaced by real detectors
%   behind the same interface -- that is why the output format is defined
%   independently of how it was produced. See docs/PERCEPTION_NOTES.md.
%
%   Effects modelled
%   ----------------
%     1. Field of view and range gating.
%     2. Occlusion: an object hidden behind a nearer, wider object at a
%        similar bearing is not detected. This is what produces the sudden
%        appearances that make Indian traffic hard, e.g. a motorcycle
%        emerging from behind a bus.
%     3. Range-dependent measurement noise.
%     4. Distance-dependent missed detections.
%     5. False positives at the configured clutter rate.
%     6. Class confusion for sensors that report class, weighted toward
%        confusing visually similar classes.
%
%   Inputs:
%       groundTruth - 1xM obstacle struct array (true states)
%       ego         - ego state struct from makeEgoState()
%       sensor      - one sensor struct from sensorConfig()
%       rngStream   - (optional) RandStream for reproducibility. Strongly
%                     recommended: every experiment in this project must be
%                     repeatable, and passing a seeded stream is how that is
%                     guaranteed.
%
%   Outputs:
%       dets - 1xD struct array with fields:
%              .pos        1x2 measured position (world frame)
%              .vel        1x2 measured velocity, [NaN NaN] if unmeasured
%              .class      char, measured class, 'unknown' if unclassified
%              .confidence 0..1
%              .sensor     char, name of the originating sensor
%              .truthId    true track id, or NaN for a false positive
%              .range      m, measured range from the sensor
%              .posCov     2x2 measurement covariance in the world frame
%
%   Example:
%       s    = RandStream('mt19937ar','Seed',42);
%       dets = simulateDetections(gt, ego, sc.camera, s);
%
%   Requires: base MATLAB only.
%
%   See also SENSORCONFIG, FUSEDETECTIONS, MULTIOBJECTTRACKER.

if nargin < 4 || isempty(rngStream)
    rngStream = RandStream.getGlobalStream();
end

dets = emptyDetArray();

% --- Sensor pose in the world -------------------------------------------
ct = cos(ego.heading);  st = sin(ego.heading);
sensorPos = ego.pos(:).' + [sensor.mountPos(1)*ct - sensor.mountPos(2)*st, ...
                            sensor.mountPos(1)*st + sensor.mountPos(2)*ct];
sensorYaw = ego.heading + sensor.mountYaw;

M = numel(groundTruth);

% --- Pass 1: visibility geometry ----------------------------------------
visible = false(1,M);
rangeTo = inf(1,M);
bearing = zeros(1,M);

for i = 1:M
    d   = groundTruth(i).pos(:).' - sensorPos;
    r   = hypot(d(1), d(2));
    az  = wrapToPiLocal(atan2(d(2), d(1)) - sensorYaw);

    rangeTo(i) = r;
    bearing(i) = az;
    visible(i) = r >= sensor.minRange && r <= sensor.maxRange && ...
                 abs(az) <= sensor.fov/2;
end

% --- Pass 2: occlusion ---------------------------------------------------
% An object is occluded when a nearer object subtends a bearing window that
% contains it. Angular half-width is approximated from the blocker's widest
% dimension, which is conservative for a vehicle seen side-on.
for i = 1:M
    if ~visible(i), continue; end
    for j = 1:M
        if i == j || ~visible(j), continue; end
        if rangeTo(j) >= rangeTo(i) - 0.5, continue; end   % not nearer

        blockerHalfWidth = max(groundTruth(j).width, groundTruth(j).length) / 2;
        halfAngle = atan2(blockerHalfWidth, max(rangeTo(j), 0.1));

        if abs(wrapToPiLocal(bearing(i) - bearing(j))) < halfAngle
            visible(i) = false;
            break;
        end
    end
end

% --- Pass 3: detection, noise, classification ---------------------------
[classNames, classInfo] = objectClasses();

for i = 1:M
    if ~visible(i), continue; end

    r = rangeTo(i);

    % Detection probability falls off with range.
    pd = sensor.pDetect * (1 - 0.35 * (r / sensor.maxRange));
    if rand(rngStream) > pd
        continue;                       % missed detection
    end

    % Measurement noise, growing with range.
    sigR  = sensor.rangeStd + sensor.rangeStdRate * r;
    sigAz = sensor.azimuthStd;

    rMeas  = r + sigR * randn(rngStream);
    azMeas = bearing(i) + sigAz * randn(rngStream);

    absAz = azMeas + sensorYaw;
    posMeas = sensorPos + rMeas * [cos(absAz), sin(absAz)];

    % Covariance in polar coordinates, rotated into the world frame. Range
    % and bearing errors are independent; the resulting world-frame ellipse
    % is elongated along the line of sight, which is the characteristic
    % shape of a real range-bearing sensor and matters to the tracker.
    sigCross = max(rMeas * sigAz, 1e-3);
    Rpolar   = diag([sigR^2, sigCross^2]);
    Rot      = [cos(absAz), -sin(absAz); sin(absAz), cos(absAz)];
    posCov   = Rot * Rpolar * Rot.';

    % Velocity: radar only, and only the radial component is observable.
    if isfinite(sensor.velStd)
        losDir  = [cos(absAz), sin(absAz)];
        vRadial = dot(groundTruth(i).vel(:).', losDir) + ...
                  sensor.velStd * randn(rngStream);
        velMeas = vRadial * losDir;
        radialStd = sensor.velStd;
    else
        velMeas = [NaN NaN];
        vRadial = NaN;
        losDir  = [NaN NaN];
        radialStd = NaN;
    end

    % Class: only sensors that classify report one, and they make mistakes.
    if sensor.providesClass
        [clsMeas, clsConf] = confuseClass(groundTruth(i).class, classNames, ...
                                          classInfo, r, sensor, rngStream);
    else
        clsMeas = 'unknown';
        clsConf = 0.6;
    end

    d = makeDet(posMeas, velMeas, clsMeas, clsConf, sensor.name, ...
                groundTruth(i).id, rMeas, posCov, vRadial, losDir, radialStd);
    dets(end+1) = d; %#ok<AGROW>
end

% --- Pass 4: false positives --------------------------------------------
nFalse = poissonSample(sensor.falseRate, rngStream);
for f = 1:nFalse
    rF  = sensor.minRange + rand(rngStream) * (sensor.maxRange - sensor.minRange);
    azF = (rand(rngStream) - 0.5) * sensor.fov;
    absAz = azF + sensorYaw;
    posF  = sensorPos + rF * [cos(absAz), sin(absAz)];

    d = makeDet(posF, [NaN NaN], 'unknown', 0.35, sensor.name, NaN, rF, ...
                eye(2) * (sensor.rangeStd + sensor.rangeStdRate*rF)^2, ...
                NaN, [NaN NaN], NaN);
    dets(end+1) = d; %#ok<AGROW>
end
end

% =====================================================================
function d = makeDet(pos, vel, cls, conf, sensorName, truthId, range, posCov, ...
                     radialVel, losDir, radialStd)
% .vel is kept for backward compatibility (radial speed times the line of
% sight). The tracker uses .radialVel / .losDir / .radialStd, which is the
% quantity radar genuinely measures (Phase 2).
d.pos        = pos;
d.vel        = vel;
d.class      = cls;
d.confidence = conf;
d.sensor     = sensorName;
d.truthId    = truthId;
d.range      = range;
d.posCov     = posCov;
d.radialVel  = radialVel;
d.losDir     = losDir;
d.radialStd  = radialStd;
end

% =====================================================================
function dets = emptyDetArray()
dets = struct('pos',{},'vel',{},'class',{},'confidence',{}, ...
              'sensor',{},'truthId',{},'range',{},'posCov',{}, ...
              'radialVel',{},'losDir',{},'radialStd',{});
end

% =====================================================================
function [cls, conf] = confuseClass(trueClass, classNames, classInfo, r, sensor, s)
%CONFUSECLASS Range-dependent class confusion, biased to similar sizes.
%   Confusion probability grows with range, and a mistaken label is drawn
%   preferentially from classes of similar physical size -- a distant
%   auto-rickshaw is far more likely to be called a car than a bus.

pCorrect = 0.94 - 0.30 * (r / sensor.maxRange);
if rand(s) < pCorrect
    cls  = trueClass;
    conf = min(0.99, pCorrect + 0.05);
    return;
end

ti = find(strcmp(classNames, trueClass), 1);
if isempty(ti)
    cls  = 'unknown';
    conf = 0.4;
    return;
end

trueSize = classInfo(ti).length;
sizes    = [classInfo.length];
simil    = 1 ./ (1 + abs(sizes - trueSize));
simil(ti) = 0;                        % cannot "confuse" with itself
simil(strcmp(classNames,'unknown')) = 0.15;

w = simil / sum(simil);
pick = find(cumsum(w) >= rand(s), 1, 'first');
if isempty(pick), pick = numel(classNames); end

cls  = classNames{pick};
conf = 0.45 + 0.25 * rand(s);
end

% =====================================================================
function n = poissonSample(lambda, s)
%POISSONSAMPLE Poisson draw by Knuth's method (no Statistics Toolbox).
if lambda <= 0
    n = 0;
    return;
end
L = exp(-lambda);
n = 0;
p = 1;
while true
    p = p * rand(s);
    if p <= L
        break;
    end
    n = n + 1;
    if n > 50           % guard against a pathological lambda
        break;
    end
end
end
