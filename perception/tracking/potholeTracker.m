function [tracks, tstate] = potholeTracker(dets, t, cfg, tstate)
%POTHOLETRACKER Static-landmark tracker that turns pothole detections into a map.
%
%   COMPONENT STATUS: REAL (inverse-variance landmark fusion) over SIMPLIFIED
%   detections
%
%   [tracks, tstate] = POTHOLETRACKER(dets, t, cfg, tstate) associates the
%   current pothole detections with known potholes, fuses repeated
%   observations, estimates each pothole's position, size and depth,
%   classifies its severity, and decides when it is confirmed.
%
%   Potholes do not move, so the tracker is a landmark map rather than a
%   motion tracker:
%     - association: nearest known pothole within cfg.pothole.gate metres
%     - position and depth: inverse-variance weighted running estimate, so
%       precise LiDAR measurements dominate imprecise camera ones and the
%       estimate sharpens as the vehicle approaches
%     - size: running mean
%     - severity: CLASSIFYPOTHOLESEVERITY of the fused depth estimate
%     - confirmed after cfg.pothole.confirmHits associated detections
%     - a TENTATIVE pothole not re-observed for 3 s is deleted (this is what
%       removes false positives); a CONFIRMED one is kept, since a pothole
%       does not go away when the vehicle stops looking at it
%
%   Detection state of each track: 'tentative' or 'confirmed'.
%
%   Inputs:
%       dets   - detections from detectPotholes()
%       t      - current simulation time (s)
%       cfg    - config struct from irpscConfig()
%       tstate - tracker state from the previous call, or [] to initialise
%
%   Outputs:
%       tracks - struct array of ALL current tracks, fields:
%                .id .pos .length .width .yaw .depth .depthStd .posStd
%                .hits .confirmed .state .severity .riskLevel .confidence
%                .firstSeen .lastSeen .truthId
%       tstate - updated state
%
%   Requires: base MATLAB only.
%
%   See also DETECTPOTHOLES, CONFIRMEDPOTHOLES, IRPSCPLANNER.

if isempty(tstate) || ~isstruct(tstate)
    tstate.tracks = emptyTracks();
    tstate.nextId = 1;
end
T = tstate.tracks;

for j = 1:numel(dets)
    d = dets(j);
    best = 0;
    % Gate grows with the detection's own position noise: a camera
    % detection 30 m away has ~0.7 m std, and a fixed gate split one
    % pothole into two tracks (Phase 2 fix).
    bestD = max(cfg.pothole.gate, 3 * d.posStd);
    for i = 1:numel(T)
        dd = hypot(T(i).pos(1) - d.pos(1), T(i).pos(2) - d.pos(2));
        if dd < bestD
            bestD = dd;  best = i;
        end
    end

    if best == 0
        nt = emptyTracks();
        nt(1).id        = tstate.nextId;
        nt(1).pos       = d.pos;
        nt(1).length    = d.length;
        nt(1).width     = d.width;
        nt(1).yaw       = d.yaw;
        nt(1).depth     = d.depth;
        nt(1).depthStd  = d.depthStd;
        nt(1).posStd    = d.posStd;
        nt(1).hits      = 1;
        nt(1).confirmed = false;
        nt(1).state     = 'tentative';
        nt(1).severity  = '';
        nt(1).riskLevel = 0;
        nt(1).confidence = d.confidence * 0.5;
        nt(1).firstSeen = t;
        nt(1).lastSeen  = t;
        nt(1).truthId   = d.truthId;
        tstate.nextId = tstate.nextId + 1;
        T(end+1) = nt(1); %#ok<AGROW>
        best = numel(T);
    else
        tr = T(best);
        wOld = 1 / max(tr.posStd^2, 1e-6);
        wNew = 1 / max(d.posStd^2,  1e-6);
        tr.pos    = (wOld * tr.pos + wNew * d.pos) / (wOld + wNew);
        tr.posStd = sqrt(1 / (wOld + wNew));
        zOld = 1 / max(tr.depthStd^2, 1e-8);
        zNew = 1 / max(d.depthStd^2,  1e-8);
        tr.depth    = (zOld * tr.depth + zNew * d.depth) / (zOld + zNew);
        tr.depthStd = sqrt(1 / (zOld + zNew));
        n = tr.hits;
        tr.length = (n * tr.length + d.length) / (n + 1);
        tr.width  = (n * tr.width  + d.width)  / (n + 1);
        tr.hits   = n + 1;
        tr.lastSeen = t;
        tr.confidence = min(0.99, 1 - (1 - tr.confidence) * (1 - 0.5 * d.confidence));
        if isnan(tr.truthId), tr.truthId = d.truthId; end
        T(best) = tr;
    end

    T(best) = refreshClass(T(best), cfg);
end

% Merge tracks that describe the same pothole (closer than their own size
% plus the gate), keeping the one with more support.
i = 1;
while i <= numel(T)
    j = i + 1;
    while j <= numel(T)
        sep = hypot(T(i).pos(1) - T(j).pos(1), T(i).pos(2) - T(j).pos(2));
        reach = cfg.pothole.gate + 0.5 * max([T(i).length, T(i).width, T(j).length, T(j).width]);
        if sep < reach
            if T(j).hits > T(i).hits
                keepT = T(j);  dropT = T(i);
            else
                keepT = T(i);  dropT = T(j);
            end
            w1 = keepT.hits;  w2 = dropT.hits;
            keepT.pos  = (w1 * keepT.pos + w2 * dropT.pos) / (w1 + w2);
            keepT.hits = w1 + w2;
            if isnan(keepT.truthId), keepT.truthId = dropT.truthId; end
            keepT.firstSeen = min(keepT.firstSeen, dropT.firstSeen);
            keepT.lastSeen  = max(keepT.lastSeen, dropT.lastSeen);
            keepT = refreshClass(keepT, cfg);
            T(i) = keepT;
            T(j) = [];
        else
            j = j + 1;
        end
    end
    i = i + 1;
end

% Drop stale tentative tracks (false positives are rarely re-observed).
keep = true(1, numel(T));
for i = 1:numel(T)
    if ~T(i).confirmed && (t - T(i).lastSeen) > 3.0
        keep(i) = false;
    end
end
T = T(keep);

tstate.tracks = T;
tracks = T;
end

% -------------------------------------------------------------------------
function tr = refreshClass(tr, cfg)
if tr.hits >= cfg.pothole.confirmHits
    tr.confirmed = true;
    tr.state     = 'confirmed';
end
[tr.severity, tr.riskLevel] = classifyPotholeSeverity(tr.depth, cfg);
end

function T = emptyTracks()
T = struct('id',{},'pos',{},'length',{},'width',{},'yaw',{},'depth',{}, ...
           'depthStd',{},'posStd',{},'hits',{},'confirmed',{},'state',{}, ...
           'severity',{},'riskLevel',{},'confidence',{},'firstSeen',{}, ...
           'lastSeen',{},'truthId',{});
end
