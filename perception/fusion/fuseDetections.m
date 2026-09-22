function fused = fuseDetections(detSets, cfg)
%FUSEDETECTIONS Combine camera, LiDAR and radar detections into one list.
%
%   COMPONENT STATUS: SIMPLIFIED (real algorithm, simplified association)
%
%   fused = FUSEDETECTIONS(detSets, cfg) merges detections from several
%   sensors that refer to the same physical object, producing one estimate
%   per object with better accuracy than any single sensor.
%
%   Why fuse at all
%   ---------------
%   The three sensors fail in different ways, and that is the point:
%     - the camera knows WHAT it is but is poor at HOW FAR,
%     - the LiDAR knows exactly where it is but not what it is,
%     - the radar knows how fast it is closing but only roughly where.
%   Fusing takes the class from the camera, the position from the LiDAR and
%   the velocity from the radar, which is strictly better than any one of
%   them.
%
%   Method
%   ------
%   Detections are grouped greedily by proximity, using a gate scaled by the
%   combined measurement uncertainty. Within a group, positions are combined
%   by INVERSE-COVARIANCE WEIGHTING:
%
%       P_fused = inv( sum_k inv(P_k) )
%       x_fused = P_fused * sum_k ( inv(P_k) * x_k )
%
%   This is the standard optimal linear combination for independent Gaussian
%   measurements. Practically, it means the LiDAR dominates the position
%   estimate because its covariance is smallest -- exactly the desired
%   behaviour, achieved without any hand-tuned per-sensor weights.
%
%   SIMPLIFICATION, stated honestly: association is greedy nearest-neighbour
%   rather than a global assignment, and the sensors' errors are assumed
%   independent. Both are reasonable at these object counts, and both are
%   things a production system would improve. With Sensor Fusion and
%   Tracking Toolbox available, trackerGNN or trackerJPDA would replace this
%   behind the same interface.
%
%   Inputs:
%       detSets - 1xS cell array, each cell a detection struct array from
%                 simulateDetections()
%       cfg     - config struct from irpscConfig()
%
%   Outputs:
%       fused - 1xF struct array with fields:
%               .pos        1x2 fused position
%               .vel        1x2 fused velocity, [NaN NaN] if unmeasured
%               .class      char, best available class
%               .confidence 0..1, raised when sensors agree
%               .posCov     2x2 fused covariance
%               .sensors    cellstr of contributing sensor names
%               .nSensors   how many sensors contributed
%               .truthId    true id if unambiguous, else NaN (diagnostics
%                           only -- never used by the planner)
%
%   Example:
%       fused = fuseDetections({camDets, lidarDets, radarDets}, cfg);
%
%   Requires: base MATLAB only.
%
%   See also SIMULATEDETECTIONS, MULTIOBJECTTRACKER.

% --- Flatten ------------------------------------------------------------
% NOTE: named allDets, not all. A variable named `all` would shadow the
% MATLAB builtin all() for the rest of this function, so any later use of
% all(...) here would silently become an indexing operation on the struct
% array instead of a logical reduction.
allDets = struct('pos',{},'vel',{},'class',{},'confidence',{}, ...
                 'sensor',{},'truthId',{},'range',{},'posCov',{}, ...
                 'radialVel',{},'losDir',{},'radialStd',{});
for s = 1:numel(detSets)
    ds = detSets{s};
    for i = 1:numel(ds)
        allDets(end+1) = ds(i); %#ok<AGROW>
    end
end

fused = struct('pos',{},'vel',{},'class',{},'confidence',{}, ...
               'posCov',{},'sensors',{},'nSensors',{},'truthId',{}, ...
               'radialVel',{},'losDir',{},'radialStd',{});

N = numel(allDets);
if N == 0
    return;
end

% --- Greedy grouping by gated proximity ---------------------------------
used   = false(1,N);
gateK  = cfg.fusion.gateSigma;   % gate, in std devs of the combined uncertainty

for i = 1:N
    if used(i), continue; end
    group = i;
    used(i) = true;

    for j = i+1:N
        if used(j), continue; end
        dp = allDets(i).pos - allDets(j).pos;
        Pc = allDets(i).posCov + allDets(j).posCov;
        % Mahalanobis distance under the combined covariance.
        if rcond(Pc) < 1e-12
            continue;
        end
        m2 = dp / Pc * dp.';
        if m2 <= gateK^2
            group(end+1) = j; %#ok<AGROW>
            used(j) = true;
        end
    end

    fused(end+1) = combineGroup(allDets(group)); %#ok<AGROW>
end
end

% =====================================================================
function f = combineGroup(dets)
%COMBINEGROUP Inverse-covariance fusion of one group of detections.

n = numel(dets);

% --- Position: inverse-covariance weighting -----------------------------
Iinfo = zeros(2,2);
iX    = zeros(2,1);
for k = 1:n
    P = dets(k).posCov;
    if rcond(P) < 1e-12
        P = P + 1e-6 * eye(2);
    end
    Pi    = inv(P);
    Iinfo = Iinfo + Pi;
    iX    = iX + Pi * dets(k).pos(:);
end
if rcond(Iinfo) < 1e-12
    Iinfo = Iinfo + 1e-6 * eye(2);
end
Pf = inv(Iinfo);
xf = Pf * iX;

% --- Velocity: mean of sensors that actually measured it ----------------
vSum = [0 0];
vN   = 0;
for k = 1:n
    if all(isfinite(dets(k).vel))
        vSum = vSum + dets(k).vel;
        vN   = vN + 1;
    end
end
if vN > 0
    vf = vSum / vN;
else
    vf = [NaN NaN];
end

% --- Class: highest-confidence classifying sensor wins ------------------
bestCls  = 'unknown';
bestConf = 0;
for k = 1:n
    if ~strcmp(dets(k).class, 'unknown') && dets(k).confidence > bestConf
        bestCls  = dets(k).class;
        bestConf = dets(k).confidence;
    end
end
if bestConf == 0
    bestConf = mean([dets.confidence]);
end

% --- Confidence: agreement between sensors raises it --------------------
% Two independent sensors seeing the same thing is materially stronger
% evidence than one, so the bonus is real -- but it is capped, because
% agreement between correlated errors is not independent evidence.
agreementBonus = min(0.15 * (n - 1), 0.25);
conf = min(1, bestConf + agreementBonus);

% --- Truth id: for diagnostics only -------------------------------------
ids = [dets.truthId];
ids = ids(~isnan(ids));
if isempty(ids) || numel(unique(ids)) > 1
    truthId = NaN;
else
    truthId = ids(1);
end

f.pos        = xf(:).';
f.vel        = vf;
f.class      = bestCls;
f.confidence = conf;
f.posCov     = Pf;
f.sensors    = {dets.sensor};
f.nSensors   = n;
f.truthId    = truthId;

% Radial (Doppler) velocity, passed through unchanged for the tracker's
% proper measurement update. If several radar returns fused, the most
% precise one is used.
f.radialVel = NaN;  f.losDir = [NaN NaN];  f.radialStd = NaN;
for k = 1:n
    if isfield(dets(k), 'radialVel') && isfinite(dets(k).radialVel) && ...
            (~isfinite(f.radialStd) || dets(k).radialStd < f.radialStd)
        f.radialVel = dets(k).radialVel;
        f.losDir    = dets(k).losDir;
        f.radialStd = dets(k).radialStd;
    end
end
end
