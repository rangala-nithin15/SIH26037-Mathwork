function p = vehicleParams(cfg)
%VEHICLEPARAMS Derived ego-vehicle parameters for control, dynamics, safety.
%
%   COMPONENT STATUS: REAL
%
%   p = VEHICLEPARAMS(cfg) expands the ego geometry held in cfg into the
%   derived quantities the controller, dynamics model and collision checker
%   need. Keeping this derivation in one place stops the same geometry being
%   recomputed inconsistently in several files.
%
%   Inputs:
%       cfg - struct from irpscConfig(). Optional; defaults to irpscConfig().
%
%   Outputs:
%       p - struct with fields:
%           .wheelbase      m
%           .length, .width m
%           .halfWidth      m
%           .frontOverhang  m, rear axle to front bumper
%           .rearOverhang   m, rear axle to rear bumper
%           .circumRadius   m, radius of a circle enclosing the whole footprint
%           .discRadius     m, radius of each of the three covering discs
%           .discOffsets    1x3, longitudinal offsets of the covering discs
%                           measured from the rear axle
%           .minTurnRadius  m, implied by the maximum steering angle
%           .maxCurvature   1/m, 1 / minTurnRadius
%
%   Three-disc approximation
%   ------------------------
%   Exact rectangle-versus-obstacle intersection is expensive to evaluate at
%   every trajectory sample. Covering the vehicle box with three overlapping
%   discs turns each check into three distance comparisons. The discs fully
%   contain the box, so the approximation is CONSERVATIVE: it can report a
%   clearance violation slightly early, never late. That is the correct
%   direction to err for a safety check.
%
%   Requires: base MATLAB only.
%
%   See also IRPSCCONFIG, CHECKCLEARANCE.

if nargin < 1 || isempty(cfg)
    cfg = irpscConfig();
end
e = cfg.ego;

p.wheelbase     = e.wheelbase;
p.length        = e.length;
p.width         = e.width;
p.halfWidth     = e.width / 2;
p.rearOverhang  = e.rearOverhang;
p.frontOverhang = e.length - e.rearOverhang;

segLen         = e.length / 3;
p.discRadius   = hypot(segLen/2, e.width/2);
p.discOffsets  = [-e.rearOverhang + 0.5*segLen, ...
                  -e.rearOverhang + 1.5*segLen, ...
                  -e.rearOverhang + 2.5*segLen];
p.circumRadius = hypot(e.length/2, e.width/2);

p.minTurnRadius = e.wheelbase / max(tan(e.maxSteer), 1e-6);
p.maxCurvature  = 1 / p.minTurnRadius;
end
