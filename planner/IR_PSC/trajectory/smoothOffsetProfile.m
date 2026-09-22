function dSmooth = smoothOffsetProfile(dProfile, cfg)
%SMOOTHOFFSETPROFILE Smooth a lateral-offset profile without moving its anchor.
%
%   COMPONENT STATUS: REAL
%
%   dSmooth = SMOOTHOFFSETPROFILE(dProfile, cfg) removes the 0.25 m
%   quantisation steps of the dynamic-programming solution with the moving
%   average cfg.traj.smoothWindow / smoothPasses, while keeping the first
%   station at the vehicle's actual lateral offset dProfile(1).
%
%   Why (Phase 2): a plain moving average pulls the first value towards its
%   neighbours. On the first plan of the demo the vehicle was at offset 0
%   and the profile moved towards the keep-left position 0.9 m; smoothing
%   put station 1 at 0.77 m, the trajectory's first point was then snapped
%   back to the vehicle, and the resulting kink had a curvature of 0.13 1/m
%   -- a lateral acceleration of 6 m/s^2 at 7 m/s. Feasibility correctly
%   rejected it and the car stopped before it had moved.
%
%   The smoothed deviation from the anchor is faded in with a raised-cosine
%   ramp over the first 2*smoothWindow+1 stations, so the offset leaves the
%   anchor with zero slope. Used by both GENERATETRAJECTORY and IRPSCPLANNER
%   so that speed ceilings refer to exactly the path that is driven.
%
%   Inputs:
%       dProfile - Nx1 offsets from DEFORMTRAJECTORY (dProfile(1) = anchor)
%       cfg      - config struct from irpscConfig()
%
%   Outputs:
%       dSmooth - Nx1 smoothed offsets, dSmooth(1) == dProfile(1)
%
%   Requires: base MATLAB only.
%
%   See also GENERATETRAJECTORY, SMOOTHSERIES.

dProfile = dProfile(:);
N = numel(dProfile);
if cfg.traj.smoothWindow <= 0 || N < 3
    dSmooth = dProfile;
    return;
end

d0 = dProfile(1);
dSmooth = smoothSeries(dProfile, cfg.traj.smoothWindow, cfg.traj.smoothPasses);

L = min(N, 2 * cfg.traj.smoothWindow + 1);
w = ones(N,1);
w(1:L) = 0.5 - 0.5 * cos(pi * (0:L-1).' / max(L-1, 1));
dSmooth = d0 + w .* (dSmooth - d0);
end
