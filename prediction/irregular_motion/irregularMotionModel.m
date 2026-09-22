function hyp = irregularMotionModel(obs, times, cfg, corridor)
%IRREGULARMOTIONMODEL Extra motion hypotheses for non-lane-following users.
%
%   COMPONENT STATUS: SIMPLIFIED
%
%   hyp = IRREGULARMOTIONMODEL(obs, times, cfg, corridor) generates
%   additional plausible short-term motions for road users whose behaviour a
%   single constant-velocity prediction would badly under-represent.
%
%   Why this exists
%   ---------------
%   The problem statement calls out sudden direction changes, informal
%   merging, wrong-way movement and unmarked crossing. A lane-following
%   predictor cannot express any of those, because it assumes the thing it
%   should be questioning. This model adds explicit alternative hypotheses:
%
%     'straight'   - continue as measured (always present, highest weight)
%     'cutLeft'    - swing left across the corridor
%     'cutRight'   - swing right across the corridor
%     'cross'      - move perpendicular to the corridor (pedestrian,
%                    animal, pushcart stepping into the road)
%     'decelerate' - brake hard to a stop
%
%   Hypothesis weights come from the class agility and from geometry (a
%   pedestrian standing at the road edge is far more likely to cross than
%   one walking away from it).
%
%   HONESTY LABEL: SIMPLIFIED. These weights are hand-designed heuristics,
%   not probabilities learned from data. They are interpretable and
%   defensible as a first model, and they are deliberately conservative
%   (they add hypotheses rather than removing them), but they should not be
%   presented as calibrated likelihoods. Learning them from recorded Indian
%   traffic is the natural next step and is listed as future work.
%
%   Inputs:
%       obs      - obstacle struct from makeObstacle()
%       times    - 1xT or Tx1 prediction times (s)
%       cfg      - config struct from irpscConfig()
%       corridor - corridor struct from extractCorridor(); may be [] if the
%                  corridor is unavailable, in which case crossing
%                  hypotheses are generated relative to the obstacle's own
%                  heading instead
%
%   Outputs:
%       hyp - 1xH struct array, each with:
%             .name    char, hypothesis label
%             .weight  0..1, normalised across hypotheses
%             .pos     Tx2 predicted positions for this hypothesis
%
%   Example:
%       h = irregularMotionModel(o, 0:0.2:3, cfg, corridor);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTOBSTACLES, PREDICTCONSTANTVELOCITY.

if nargin < 4, corridor = []; end

t = times(:);
T = numel(t);

basePos = predictConstantVelocity(obs, t);

% --- Reference direction for "sideways" ------------------------------
% Prefer the corridor heading: sideways means across the ROAD, not across
% the obstacle's own axis. Falls back to the obstacle heading when no
% corridor is available.
if ~isempty(corridor) && isfield(corridor, 'center') && size(corridor.center,1) >= 2
    [~, ~, idx] = projectPointOnPath(corridor.center, obs.pos);
    refHeading  = corridor.heading(min(idx, numel(corridor.heading)));
else
    refHeading = obs.heading;
end
leftDir = [-sin(refHeading), cos(refHeading)];

agility = obs.agility;

% --- Build hypotheses -------------------------------------------------
names   = {'straight'};
weights = 1.0;
posList = {basePos};

if agility > 0.2
    % Lateral cut across the corridor. Displacement grows quadratically:
    % a real swerve starts slowly and builds.
    cutSpeed = 0.6 + 2.0 * agility;             % m/s of lateral drift
    lateral  = 0.5 * cutSpeed * t.^2 / max(t(end), eps);

    names{end+1}   = 'cutLeft';
    posList{end+1} = basePos + lateral .* leftDir;
    weights(end+1) = 0.30 * agility;

    names{end+1}   = 'cutRight';
    posList{end+1} = basePos - lateral .* leftDir;
    weights(end+1) = 0.30 * agility;
end

% --- Crossing hypothesis ---------------------------------------------
% Applies to road users that step into the road from the side. Weighted up
% when the obstacle is slow AND near a corridor boundary, which is exactly
% the unmarked-pedestrian-crossing situation.
if any(strcmp(obs.class, {'pedestrian','animal','pushcart','bicycle'}))
    crossSpeed = min(obs.speed + 0.8, 2.5);
    crossDisp  = crossSpeed * t;

    nearEdge = 1.0;
    if ~isempty(corridor) && isfield(corridor, 'center') && size(corridor.center,1) >= 2
        [~, dLat] = projectPointOnPath(corridor.center, obs.pos);
        halfW     = max(mean(corridor.width) / 2, 0.5);
        % 1 at the edge, ~0 in the middle of the road.
        nearEdge  = min(1, abs(dLat) / halfW);
        sideSign  = -sign(dLat);          % cross toward the road centre
        if sideSign == 0, sideSign = 1; end
    else
        sideSign = 1;
    end

    names{end+1}   = 'cross';
    posList{end+1} = basePos + sideSign * crossDisp .* leftDir;
    weights(end+1) = 0.45 * nearEdge;
end

% --- Hard deceleration ------------------------------------------------
if obs.speed > 1.0
    decel   = 3.0;
    tStop   = obs.speed / decel;
    dTrav   = obs.speed * min(t, tStop) - 0.5 * decel * min(t, tStop).^2;
    fwd     = [cos(obs.heading), sin(obs.heading)];
    names{end+1}   = 'decelerate';
    posList{end+1} = obs.pos(:).' + dTrav .* fwd;
    weights(end+1) = 0.20;
end

weights = weights / sum(weights);

hyp = struct('name', names, 'weight', num2cell(weights), 'pos', posList);
end
