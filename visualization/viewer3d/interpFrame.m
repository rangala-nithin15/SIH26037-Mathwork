function f = interpFrame(fA, fB, u)
%INTERPFRAME Visual interpolation between two simulation states.
%
%   COMPONENT STATUS: REAL (visualisation only).
%
%   f = INTERPFRAME(fA, fB, u) returns a frame whose ego pose and road-user
%   poses lie a fraction u (0..1) of the way from state fA to state fB.
%   Everything else -- the planned trajectory, corridor, risk grid,
%   predictions, tracks, decisions, pothole tracks -- is taken unchanged
%   from fA.
%
%   WHAT THIS IS AND IS NOT: this smooths the PICTURE between two states the
%   simulation already produced, so a vehicle does not appear to teleport
%   when the display runs faster than the state rate. It creates no new
%   simulation state, changes no logged value, and is never fed back into
%   the planner or the vehicle model. With u = 0 the frame is exactly fA.
%
%   The returned frame carries .tween = true (when 0 < u < 1) so the
%   renderer can update only the layers that moved.
%
%   Requires: base MATLAB only.
%
%   See also REPLAYDEMO, RUNDEMO, UPDATEDEMOVIEWER.

f = fA;
if u <= 0 || isempty(fB)
    f.tween = false;
    return;
end
u = min(1, u);
f.tween = (u < 1);

% -- ego pose ---------------------------------------------------------
f.ego.pos     = (1-u) * fA.ego.pos + u * fB.ego.pos;
f.ego.heading = fA.ego.heading + u * wrapToPiLocal(fB.ego.heading - fA.ego.heading);
f.ego.speed   = (1-u) * fA.ego.speed + u * fB.ego.speed;
if isfield(fA.ego, 'steer') && isfield(fB.ego, 'steer')
    f.ego.steer = (1-u) * fA.ego.steer + u * fB.ego.steer;
end
f.t = (1-u) * fA.t + u * fB.t;

% -- road users, matched by id ---------------------------------------
if ~isempty(fA.truth) && ~isempty(fB.truth)
    idB = [fB.truth.id];
    for i = 1:numel(fA.truth)
        j = find(idB == fA.truth(i).id, 1);
        if isempty(j), continue; end
        f.truth(i).pos     = (1-u) * fA.truth(i).pos + u * fB.truth(j).pos;
        f.truth(i).heading = fA.truth(i).heading + ...
                             u * wrapToPiLocal(fB.truth(j).heading - fA.truth(i).heading);
    end
end
end
