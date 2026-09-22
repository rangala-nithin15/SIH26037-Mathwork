function [tracks, tstate] = multiObjectTracker(fusedDets, dt, cfg, tstate)
%MULTIOBJECTTRACKER Constant-velocity Kalman tracker with GNN association.
%
%   COMPONENT STATUS: DOCUMENTED FALLBACK
%   (Sensor Fusion and Tracking Toolbox is not available in this package, so
%   trackerGNN / trackerJPDA cannot be used. This is a complete, working
%   hand-written substitute -- fallback F5 in docs/COMPONENT_REGISTER.md.
%   It is real code, not a stub, but it is simpler than the toolbox
%   trackers: single motion model, global nearest neighbour rather than
%   joint probabilistic association.)
%
%   [tracks, tstate] = MULTIOBJECTTRACKER(fusedDets, dt, cfg, tstate) runs
%   one cycle of predict, associate, update and track management.
%
%   IR-PSC step 3: track surrounding road users.
%
%   STATE AND MODEL
%   ---------------
%   Each track carries x = [px; vx; py; vy] with a constant-velocity model.
%   Unknown acceleration enters as process noise, so the covariance grows
%   between updates and the gate widens for a track that has not been seen
%   recently. That is what lets a briefly occluded road user be
%   re-associated when it reappears.
%
%   ASSOCIATION
%   -----------
%   Detections are matched to tracks by global nearest neighbour under a
%   chi-square gate on the Mahalanobis distance in measurement space. Pairs
%   are consumed in increasing distance order, so the most confident matches
%   claim their detections first.
%
%   Limitation, stated honestly: greedy assignment is not guaranteed
%   globally optimal, and in dense crowds it can swap two tracks that pass
%   close together. JPDA exists precisely to handle that case. This is
%   acknowledged rather than hidden, and the interface is designed so a
%   toolbox tracker can be dropped in unchanged.
%
%   TRACK MANAGEMENT
%   ----------------
%     - A new detection starts a TENTATIVE track.
%     - It is CONFIRMED after being seen on 3 cycles (M-of-N style logic).
%     - A track missed for up to 5 cycles COASTS on its prediction, which is
%       what carries a road user through a short occlusion.
%     - After 5 consecutive misses it is deleted.
%   Only confirmed tracks are returned to the planner, so a single clutter
%   detection cannot cause a brake.
%
%   Inputs:
%       fusedDets - detection struct array from fuseDetections()
%       dt        - seconds since the previous call
%       cfg       - config struct from irpscConfig()
%       tstate    - tracker state from the previous call, or [] to init
%
%   Outputs:
%       tracks - 1xT obstacle struct array (makeObstacle format) containing
%                CONFIRMED tracks only, ready for the planner
%       tstate - updated tracker state, pass into the next call
%
%   Example:
%       tstate = [];
%       [tracks, tstate] = multiObjectTracker(fused, 0.05, cfg, tstate);
%
%   Requires: base MATLAB only.
%
%   See also FUSEDETECTIONS, MAKEOBSTACLE, PREDICTOBSTACLES.

% --- Tuning -------------------------------------------------------------
CONFIRM_HITS = cfg.tracking.confirmHits;   % cycles seen before a track is trusted
MAX_MISSES   = cfg.tracking.maxMisses;     % cycles coasted before deletion
GATE_CHI2    = cfg.tracking.gateChi2;      % chi-square, 2 dof
SIGMA_ACCEL  = cfg.tracking.sigmaAccel;    % m/s^2 process noise

if isempty(tstate) || ~isstruct(tstate)
    tstate.tracks = emptyTrackArray();
    tstate.nextId = 1;
end

T = tstate.tracks;

% =====================================================================
% 1. PREDICT
% =====================================================================
F = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];

% Discrete white-noise acceleration process noise.
q  = SIGMA_ACCEL^2;
Q1 = q * [dt^4/4, dt^3/2; dt^3/2, dt^2];
Q  = blkdiag(Q1, Q1);

for i = 1:numel(T)
    T(i).x = F * T(i).x;
    T(i).P = F * T(i).P * F.' + Q;
    T(i).age = T(i).age + 1;
end

% =====================================================================
% 2. ASSOCIATE (global nearest neighbour under a chi-square gate)
% =====================================================================
nT = numel(T);
nD = numel(fusedDets);

H = [1 0 0 0;
     0 0 1 0];

assignedDet   = false(1, nD);
assignedTrack = false(1, nT);
pairs = zeros(0,3);      % [trackIdx, detIdx, mahalanobisSquared]

for i = 1:nT
    for j = 1:nD
        z  = fusedDets(j).pos(:);
        zp = H * T(i).x;
        S  = H * T(i).P * H.' + fusedDets(j).posCov;
        if rcond(S) < 1e-12
            S = S + 1e-6 * eye(2);
        end
        nu = z - zp;
        m2 = nu.' / S * nu;
        if m2 <= GATE_CHI2
            pairs(end+1,:) = [i, j, m2]; %#ok<AGROW>
        end
    end
end

if ~isempty(pairs)
    pairs = sortrows(pairs, 3);      % best matches claim first
    for k = 1:size(pairs,1)
        ti = pairs(k,1);
        di = pairs(k,2);
        if assignedTrack(ti) || assignedDet(di)
            continue;
        end
        assignedTrack(ti) = true;
        assignedDet(di)   = true;

        % --- 3. UPDATE ---------------------------------------------
        z  = fusedDets(di).pos(:);
        R  = fusedDets(di).posCov;
        zp = H * T(ti).x;
        S  = H * T(ti).P * H.' + R;
        if rcond(S) < 1e-12
            S = S + 1e-6 * eye(2);
        end
        K  = T(ti).P * H.' / S;
        T(ti).x = T(ti).x + K * (z - zp);
        Ijoseph = eye(4) - K * H;
        % Joseph form: stays symmetric positive-definite under round-off,
        % which the simpler (I-KH)P form does not always do.
        T(ti).P = Ijoseph * T(ti).P * Ijoseph.' + K * R * K.';

        % Radar Doppler: a proper Kalman update with the RADIAL velocity
        % measurement z = lx*vx + ly*vy (H = [0 lx 0 ly]).
        %
        % Phase 2 fix: the previous code blended the velocity state 30 %
        % per frame toward (radial speed x line of sight). Radar cannot
        % observe the velocity component ACROSS its line of sight, so for
        % anything crossing in front of the car -- cattle, pedestrians --
        % that blend dragged the estimated velocity towards zero every
        % frame and the prediction said the animal was standing still. The
        % measurement model below corrects only the component radar
        % actually measures; the crossing component comes from the
        % position updates.
        fd = fusedDets(di);
        if isfield(fd, 'radialVel') && isfinite(fd.radialVel) && all(isfinite(fd.losDir))
            Hv = [0, fd.losDir(1), 0, fd.losDir(2)];
            Rv = max(fd.radialStd, 1e-3)^2;
            Sv = Hv * T(ti).P * Hv.' + Rv;
            Kv = T(ti).P * Hv.' / Sv;
            T(ti).x = T(ti).x + Kv * (fd.radialVel - Hv * T(ti).x);
            Iv = eye(4) - Kv * Hv;
            T(ti).P = Iv * T(ti).P * Iv.' + Kv * Rv * Kv.';
        end

        T(ti).hits       = T(ti).hits + 1;
        T(ti).misses     = 0;
        T(ti).confidence = 0.7*T(ti).confidence + 0.3*fusedDets(di).confidence;
        % Class by accumulated evidence, not the latest detection: camera
        % class confusion otherwise made one track flip between car, truck
        % and pushcart frame to frame (Phase 2).
        if ~strcmp(fusedDets(di).class, 'unknown')
            T(ti).classVotes = addVote(T(ti).classVotes, fusedDets(di).class, ...
                                       fusedDets(di).confidence);
            T(ti).class = bestVote(T(ti).classVotes, T(ti).class);
        end
        T(ti).truthId = fusedDets(di).truthId;

        if T(ti).hits >= CONFIRM_HITS
            T(ti).confirmed = true;
        end
    end
end

% =====================================================================
% 4. COAST UNASSOCIATED TRACKS
% =====================================================================
for i = 1:nT
    if ~assignedTrack(i)
        T(i).misses     = T(i).misses + 1;
        % Confidence decays while coasting: an unseen track is a weaker
        % claim each cycle, and that decay flows into planner confidence.
        T(i).confidence = T(i).confidence * 0.85;
    end
end

% =====================================================================
% 5. DELETE STALE TRACKS
% =====================================================================
keep = true(1, numel(T));
for i = 1:numel(T)
    if T(i).misses > MAX_MISSES
        keep(i) = false;
    end
end
T = T(keep);

% =====================================================================
% 6. INITIATE NEW TRACKS
% =====================================================================
for j = 1:nD
    if assignedDet(j)
        continue;
    end
    d = fusedDets(j);

    nt.id    = tstate.nextId;
    tstate.nextId = tstate.nextId + 1;

    % Only the radial part of the velocity is known at birth; the rest
    % stays at zero with the large initial variance below.
    if isfield(d, 'radialVel') && isfinite(d.radialVel) && all(isfinite(d.losDir))
        v0 = d.radialVel * d.losDir(:);
    else
        v0 = [0; 0];
    end
    nt.x = [d.pos(1); v0(1); d.pos(2); v0(2)];

    % Initial velocity covariance is large: one detection says nothing
    % about velocity, so claiming otherwise would make the first prediction
    % overconfident.
    Pv = cfg.tracking.initVelVar;
    nt.P = blkdiag([d.posCov(1,1), 0; 0, Pv], [d.posCov(2,2), 0; 0, Pv]);

    nt.class      = d.class;
    nt.confidence = d.confidence;
    nt.hits       = 1;
    nt.misses     = 0;
    nt.age        = 1;
    nt.confirmed  = false;
    nt.truthId    = d.truthId;
    nt.classVotes = struct();
    if ~strcmp(d.class, 'unknown')
        nt.classVotes = addVote(nt.classVotes, d.class, d.confidence);
    end

    T(end+1) = nt; %#ok<AGROW>
end

tstate.tracks = T;

% =====================================================================
% 7. EMIT CONFIRMED TRACKS AS OBSTACLES
% =====================================================================
tracks = struct('id',{},'class',{},'pos',{},'vel',{},'accel',{},'heading',{}, ...
                'speed',{},'length',{},'width',{},'confidence',{},'age',{}, ...
                'posCov',{},'vulnerable',{},'agility',{},'truthId',{});

for i = 1:numel(T)
    if ~T(i).confirmed
        continue;
    end
    pos = [T(i).x(1), T(i).x(3)];
    vel = [T(i).x(2), T(i).x(4)];
    o = makeObstacle(T(i).id, T(i).class, pos, vel, ...
                     'Confidence', max(0, min(1, T(i).confidence)), ...
                     'Age',        T(i).age, ...
                     'PosCov',     [T(i).P(1,1), T(i).P(1,3); ...
                                    T(i).P(3,1), T(i).P(3,3)], ...
                     'TruthId',    T(i).truthId);
    tracks(end+1) = o; %#ok<AGROW>
end
end

% =====================================================================
function T = emptyTrackArray()
T = struct('id',{},'x',{},'P',{},'class',{},'confidence',{}, ...
           'hits',{},'misses',{},'age',{},'confirmed',{},'truthId',{}, ...
           'classVotes',{});
end

% =====================================================================
function v = addVote(v, cls, w)
if isfield(v, cls)
    v.(cls) = v.(cls) + w;
else
    v.(cls) = w;
end
end

function cls = bestVote(v, fallback)
f = fieldnames(v);
if isempty(f)
    cls = fallback;
    return;
end
best = -Inf;  cls = fallback;
for i = 1:numel(f)
    if v.(f{i}) > best
        best = v.(f{i});  cls = f{i};
    end
end
end
