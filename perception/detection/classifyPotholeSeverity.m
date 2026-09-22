function [severity, riskLevel] = classifyPotholeSeverity(depth, cfg)
%CLASSIFYPOTHOLESEVERITY Map an (estimated) pothole depth to a severity class.
%
%   COMPONENT STATUS: REAL (rule), thresholds are engineering judgement
%
%   [severity, riskLevel] = CLASSIFYPOTHOLESEVERITY(depth, cfg) returns
%   'minor', 'moderate' or 'severe' using cfg.pothole.depthModerate and
%   cfg.pothole.depthSevere, and the matching display/decision risk level
%   from cfg.pothole.risk.
%
%   The same function classifies ground truth (for evaluation) and the
%   tracker's depth ESTIMATE (for planning), so a severity mistake in the
%   simulation is always a perception error, never a definition mismatch.
%
%   Thresholds are NOT taken from any road standard; they are stated
%   assumptions in irpscConfig and must be presented as such.
%
%   Inputs:
%       depth - m, scalar
%       cfg   - config struct from irpscConfig()
%
%   Outputs:
%       severity  - char
%       riskLevel - scalar in [0,1]
%
%   Requires: base MATLAB only.
%
%   See also MAKEPOTHOLE, POTHOLETRACKER.

if depth >= cfg.pothole.depthSevere
    severity = 'severe';
elseif depth >= cfg.pothole.depthModerate
    severity = 'moderate';
else
    severity = 'minor';
end
riskLevel = cfg.pothole.risk.(severity);
end
