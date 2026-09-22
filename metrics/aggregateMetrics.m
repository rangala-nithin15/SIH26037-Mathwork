function A = aggregateMetrics(metricList, label)
%AGGREGATEMETRICS Summarise a metric set across several runs.
%
%   COMPONENT STATUS: REAL
%
%   A = AGGREGATEMETRICS(metricList, label) combines the per-run metric
%   structs from COMPUTEMETRICS into one summary.
%
%   For every numeric field the mean, standard deviation, minimum, maximum
%   and sample count are reported.
%
%   Why the standard deviation and the count are not optional
%   ---------------------------------------------------------
%   A mean from three runs and a mean from three hundred are not comparable
%   claims, and a mean without a spread hides whether a difference between
%   two planners is real or is noise. Reporting a bare average across runs
%   is the most common way a comparison becomes misleading without anyone
%   intending it to, so this function makes the spread and the sample size
%   impossible to omit.
%
%   Safety-critical fields are additionally summarised by their WORST value
%   rather than their mean, because the mean of a minimum clearance is not
%   a safety statement -- the worst case is.
%
%   Inputs:
%       metricList - 1xR struct array or cell array from computeMetrics()
%       label      - char, a name for this set (e.g. 'IR-PSC / village')
%
%   Outputs:
%       A - struct with fields:
%           .label     char
%           .nRuns     number of runs aggregated
%           .fields    cellstr of the fields summarised
%           .mean, .std, .min, .max  structs keyed by field name
%           .worstMinClearance, .totalCollisions, .completionRate
%           .provenance  copied from the first run
%
%   Example:
%       A = aggregateMetrics(allMetrics, 'IR-PSC / market');
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, RUNCOMPARISON.

if iscell(metricList)
    tmp = metricList;
    metricList = tmp{1};
    for i = 2:numel(tmp)
        metricList(i) = tmp{i};
    end
end

R = numel(metricList);
A.label = label;
A.nRuns = R;

if R == 0
    A.fields = {};
    A.mean = struct(); A.std = struct(); A.min = struct(); A.max = struct();
    A.worstMinClearance = NaN;
    A.totalCollisions   = 0;
    A.completionRate    = NaN;
    A.provenance        = struct();
    return;
end

names = fieldnames(metricList(1));
numericFields = {};

for k = 1:numel(names)
    v = metricList(1).(names{k});
    if isnumeric(v) && isscalar(v)
        numericFields{end+1} = names{k}; %#ok<AGROW>
    elseif islogical(v) && isscalar(v)
        numericFields{end+1} = names{k}; %#ok<AGROW>
    end
end

A.fields = numericFields;
for k = 1:numel(numericFields)
    f = numericFields{k};
    vals = zeros(1,R);
    for r = 1:R
        vals(r) = double(metricList(r).(f));
    end
    finiteVals = vals(isfinite(vals));
    if isempty(finiteVals)
        A.mean.(f) = NaN; A.std.(f) = NaN;
        A.min.(f)  = NaN; A.max.(f) = NaN;
    else
        A.mean.(f) = mean(finiteVals);
        if numel(finiteVals) > 1
            A.std.(f) = std(finiteVals);
        else
            A.std.(f) = 0;
        end
        A.min.(f) = min(finiteVals);
        A.max.(f) = max(finiteVals);
    end
end

% --- Safety fields summarised by worst case, not by average -------------
if isfield(A.min, 'minClearance')
    A.worstMinClearance = A.min.minClearance;
else
    A.worstMinClearance = NaN;
end

if isfield(A.mean, 'collisionCount')
    A.totalCollisions = A.mean.collisionCount * R;
else
    A.totalCollisions = 0;
end

if isfield(A.mean, 'completed')
    A.completionRate = A.mean.completed;
else
    A.completionRate = NaN;
end

if isfield(metricList(1), 'provenance')
    A.provenance = metricList(1).provenance;
else
    A.provenance = struct();
end
end
