function dets = detectPotholes(potholes, ego, groundTruth, cfg, rngStream)
%DETECTPOTHOLES Simulated camera + LiDAR detection of road-surface potholes.
%
%   COMPONENT STATUS: SIMPLIFIED (geometric/statistical sensor model)
%
%   dets = DETECTPOTHOLES(potholes, ego, groundTruth, cfg, rngStream)
%   returns noisy pothole detections from two simulated sensors.
%
%   HONESTY NOTE -- read before quoting anything from this
%   ------------------------------------------------------
%   No image and no point cloud is processed, and no pothole detector has
%   been trained. This function turns ground-truth potholes into plausible
%   detections with range-dependent detection probability, position, size
%   and depth noise, occlusion by road users, and false positives. It lets
%   the planning pipeline be exercised end to end with realistic
%   imperfection. It says nothing about how well a real detector would do.
%
%   Sensor models (values are assumptions, not datasheet figures)
%   -------------------------------------------------------------
%   CAMERA  (road-surface appearance)
%       range <= 30 m, FOV 90 deg. Detection probability falls with the
%       square of range and rises with pothole area. Position noise grows
%       with range (monocular depth is poor). Depth is only weakly
%       observable from appearance, so the camera's depth estimate carries
%       40 % noise. False positives (shadows, patches) at a low rate.
%   LIDAR   (ground returns)
%       range <= 20 m, FOV 120 deg. Ground returns become sparse at grazing
%       angles, hence the short range. Detection probability rises with
%       depth (a 2 cm dip is hard to see, a 10 cm hole is not). Position
%       noise ~5 cm, depth noise ~1 cm.
%
%   Occlusion: a pothole is not seen by a sensor if a road user stands
%   between the sensor and the pothole within the road user's angular width.
%
%   Inputs:
%       potholes    - ground-truth pothole array from makePothole()
%       ego         - ego state (rear axle pose)
%       groundTruth - current road users (for occlusion), may be empty
%       cfg         - config struct from irpscConfig()
%       rngStream   - RandStream, the run's single random stream
%
%   Outputs:
%       dets - struct array: .pos .length .width .yaw .depth .depthStd
%              .posStd .confidence .sensor .truthId (NaN for false positives)
%
%   Requires: base MATLAB only.
%
%   See also POTHOLETRACKER, MAKEPOTHOLE, SIMULATEDETECTIONS.

dets = struct('pos',{},'length',{},'width',{},'yaw',{},'depth',{}, ...
              'depthStd',{},'posStd',{},'confidence',{},'sensor',{},'truthId',{});

sensors = potholeSensorModels();
ct = cos(ego.heading);  st = sin(ego.heading);

for sIdx = 1:numel(sensors)
    sm = sensors(sIdx);
    sPos = ego.pos(:).' + [sm.mount(1)*ct - sm.mount(2)*st, sm.mount(1)*st + sm.mount(2)*ct];
    sYaw = ego.heading;

    for p = 1:numel(potholes)
        ph  = potholes(p);
        d   = ph.pos - sPos;
        r   = hypot(d(1), d(2));
        az  = wrapToPiLocal(atan2(d(2), d(1)) - sYaw);
        if r > sm.maxRange || r < sm.minRange || abs(az) > sm.fov/2
            continue;
        end
        if isOccluded(sPos, r, atan2(d(2), d(1)), groundTruth)
            continue;
        end

        area = pi * ph.length * ph.width / 4;
        switch sm.name
            case 'camera'
                pd = sm.pDetect * (1 - (r / sm.maxRange)^2) * min(1, sqrt(area / 0.4));
            otherwise
                pd = sm.pDetect * (1 - 0.5 * r / sm.maxRange) * min(1, ph.depth / 0.04);
        end
        if rand(rngStream) > pd
            continue;
        end

        posStd = sm.posStd0 + sm.posStdRate * r;
        dets(end+1) = struct( ...
            'pos',        ph.pos + posStd * [randn(rngStream), randn(rngStream)], ...
            'length',     max(0.1, ph.length * (1 + sm.sizeNoise * randn(rngStream))), ...
            'width',      max(0.1, ph.width  * (1 + sm.sizeNoise * randn(rngStream))), ...
            'yaw',        ph.yaw + 0.1 * randn(rngStream), ...
            'depth',      max(0.005, ph.depth + sm.depthNoise(ph.depth) * randn(rngStream)), ...
            'depthStd',   sm.depthNoise(ph.depth), ...
            'posStd',     posStd, ...
            'confidence', min(0.95, 0.5 + 0.5 * pd), ...
            'sensor',     sm.name, ...
            'truthId',    ph.id); %#ok<AGROW>
    end

    % False positives on the road ahead (shadows, patched asphalt).
    if sm.falseRate > 0 && rand(rngStream) < sm.falseRate
        rF  = sm.minRange + rand(rngStream) * (sm.maxRange - sm.minRange);
        azF = (rand(rngStream) - 0.5) * sm.fov * 0.5;
        pF  = sPos + rF * [cos(sYaw + azF), sin(sYaw + azF)];
        dets(end+1) = struct('pos', pF, 'length', 0.6, 'width', 0.5, 'yaw', 0, ...
            'depth', 0.02 + 0.03 * rand(rngStream), 'depthStd', 0.03, ...
            'posStd', sm.posStd0 + sm.posStdRate * rF, 'confidence', 0.35, ...
            'sensor', sm.name, 'truthId', NaN); %#ok<AGROW>
    end
end
end

% -------------------------------------------------------------------------
function sm = potholeSensorModels()
sm(1).name       = 'camera';
sm(1).mount      = [2.0, 0];
sm(1).minRange   = 2.0;
sm(1).maxRange   = 30.0;
sm(1).fov        = deg2rad(90);
sm(1).pDetect    = 0.85;
sm(1).posStd0    = 0.10;
sm(1).posStdRate = 0.02;
sm(1).sizeNoise  = 0.15;
sm(1).depthNoise = @(dd) 0.4 * dd + 0.01;
sm(1).falseRate  = 0.01;

sm(2).name       = 'lidar';
sm(2).mount      = [1.5, 0];
sm(2).minRange   = 1.0;
sm(2).maxRange   = 20.0;
sm(2).fov        = deg2rad(120);
sm(2).pDetect    = 0.90;
sm(2).posStd0    = 0.05;
sm(2).posStdRate = 0.002;
sm(2).sizeNoise  = 0.08;
sm(2).depthNoise = @(dd) 0.01;
sm(2).falseRate  = 0.002;
end

function occ = isOccluded(sPos, r, bearing, gt)
occ = false;
for k = 1:numel(gt)
    dv = gt(k).pos - sPos;
    rk = hypot(dv(1), dv(2));
    if rk >= r - 0.5 || rk < 0.5
        continue;
    end
    half = atan2(max(gt(k).length, gt(k).width) / 2, rk);
    if abs(wrapToPiLocal(atan2(dv(2), dv(1)) - bearing)) < half
        occ = true;
        return;
    end
end
end
