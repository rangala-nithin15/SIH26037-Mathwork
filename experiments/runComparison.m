function results = runComparison(scenarioNames, seeds, opts)
%RUNCOMPARISON Baseline versus IR-PSC, under identical conditions.
%
%   COMPONENT STATUS: REAL
%
%   results = RUNCOMPARISON(scenarioNames, seeds, opts) runs both planners
%   over the same scenarios with the same seeds and returns the comparison.
%
%   ============================================================
%   WHY IDENTICAL SEEDS ARE NOT NEGOTIABLE
%   ============================================================
%   Both planners must face exactly the same world: the same missed
%   detections, the same false positives, the same measurement noise, the
%   same actor timings. Otherwise a difference in results could simply be a
%   difference in luck, and the comparison would prove nothing.
%
%   This function enforces that by construction: for each (scenario, seed)
%   pair it runs both planners with that identical seed. It is not possible
%   to accidentally compare across different noise realisations.
%
%   ============================================================
%   REPORT THE RESULT YOU GET
%   ============================================================
%   This function computes and returns whatever the runs produce. If the
%   baseline wins on some metric, that appears in the output unchanged. A
%   comparison that can only come out one way is not a comparison, and a
%   finding that IR-PSC does not help in some scenario is a legitimate and
%   publishable result -- report it.
%
%   No result is written into this file. Nothing is returned that was not
%   computed from an executed run.
%
%   Inputs:
%       scenarioNames - cellstr of scenarios, or [] for all five
%       seeds         - vector of integer seeds, or [] for 1:5
%       opts          - (optional) struct passed through to runScenario,
%                       plus:
%                       .verbose logical, print a progress line per run
%                       .saveDir char, folder for saved .mat results
%
%   Outputs:
%       results - struct with fields:
%           .scenarios   cellstr of scenarios run
%           .seeds       the seeds used
%           .irpsc       struct array of per-run metrics for IR-PSC
%           .baseline    struct array of per-run metrics for the baseline
%           .irpscAgg    per-scenario aggregates for IR-PSC
%           .baselineAgg per-scenario aggregates for the baseline
%           .summary     table-like struct of the headline comparison
%           .timestamp   when the comparison was run
%
%   Example:
%       r = runComparison({'village','market'}, 1:10, struct('verbose',true));
%       printComparison(r);
%
%   Requires: base MATLAB only.
%
%   See also RUNSCENARIO, RUNABLATION, AGGREGATEMETRICS, PRINTCOMPARISON.

if nargin < 1 || isempty(scenarioNames)
    scenarioNames = {'village','urban','highway','market','cattle'};
end
if nargin < 2 || isempty(seeds)
    seeds = 1:5;
end
if nargin < 3, opts = struct(); end

verbose = isfield(opts,'verbose') && opts.verbose;

nS = numel(scenarioNames);
nR = numel(seeds);

irpscAll    = cell(nS, nR);
baselineAll = cell(nS, nR);

for si = 1:nS
    for ri = 1:nR
        runOpts = opts;
        runOpts.seed = seeds(ri);

        % Both planners get the SAME seed. This is the whole point.
        [~, Mi] = runScenario(scenarioNames{si}, @irpscPlanner,    [], runOpts);
        [~, Mb] = runScenario(scenarioNames{si}, @baselinePlanner, [], runOpts);

        irpscAll{si, ri}    = Mi;
        baselineAll{si, ri} = Mb;

        if verbose
            fprintf(['%-10s seed %3d | IR-PSC: clear %.2f m, coll %d, ' ...
                     'avg %.2f m/s | Baseline: clear %.2f m, coll %d, avg %.2f m/s\n'], ...
                    scenarioNames{si}, seeds(ri), ...
                    Mi.minClearance, Mi.collisionCount, Mi.averageSpeed, ...
                    Mb.minClearance, Mb.collisionCount, Mb.averageSpeed);
        end
    end
end

% --- Aggregate per scenario ---------------------------------------------
irpscAgg    = cell(1, nS);
baselineAgg = cell(1, nS);
for si = 1:nS
    irpscAgg{si}    = aggregateMetrics(irpscAll(si,:), ...
                        sprintf('IR-PSC / %s', scenarioNames{si}));
    baselineAgg{si} = aggregateMetrics(baselineAll(si,:), ...
                        sprintf('Baseline / %s', scenarioNames{si}));
end

results.scenarios   = scenarioNames;
results.seeds       = seeds;
results.irpsc       = irpscAll;
results.baseline    = baselineAll;
results.irpscAgg    = irpscAgg;
results.baselineAgg = baselineAgg;
results.timestamp   = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
results.nRuns       = nS * nR * 2;

% --- Headline summary ----------------------------------------------------
results.summary = buildSummary(scenarioNames, irpscAgg, baselineAgg);

% --- Optional save -------------------------------------------------------
if isfield(opts,'saveDir') && ~isempty(opts.saveDir)
    if exist(opts.saveDir, 'dir') ~= 7
        mkdir(opts.saveDir);
    end
    fname = fullfile(opts.saveDir, ...
                     sprintf('comparison_%s.mat', datestr(now,'yyyymmdd_HHMMSS'))); %#ok<TNOW1,DATST>
    save(fname, 'results');
    fprintf('Saved comparison to %s\n', fname);
end
end

% =====================================================================
function S = buildSummary(scenarioNames, irpscAgg, baselineAgg)
%BUILDSUMMARY Headline metrics side by side, per scenario.
nS = numel(scenarioNames);
S.scenario           = scenarioNames;
S.irpscMinClearance    = nan(1,nS);
S.baselineMinClearance = nan(1,nS);
S.irpscCollisions      = nan(1,nS);
S.baselineCollisions   = nan(1,nS);
S.irpscCompletion      = nan(1,nS);
S.baselineCompletion   = nan(1,nS);
S.irpscAvgSpeed        = nan(1,nS);
S.baselineAvgSpeed     = nan(1,nS);
S.irpscSmoothness      = nan(1,nS);
S.baselineSmoothness   = nan(1,nS);
S.irpscFailures        = nan(1,nS);
S.baselineFailures     = nan(1,nS);

for i = 1:nS
    a = irpscAgg{i};
    b = baselineAgg{i};
    S.irpscMinClearance(i)    = a.worstMinClearance;
    S.baselineMinClearance(i) = b.worstMinClearance;
    S.irpscCollisions(i)      = safeGet(a.mean, 'collisionCount');
    S.baselineCollisions(i)   = safeGet(b.mean, 'collisionCount');
    S.irpscCompletion(i)      = a.completionRate;
    S.baselineCompletion(i)   = b.completionRate;
    S.irpscAvgSpeed(i)        = safeGet(a.mean, 'averageSpeed');
    S.baselineAvgSpeed(i)     = safeGet(b.mean, 'averageSpeed');
    S.irpscSmoothness(i)      = safeGet(a.mean, 'smoothness');
    S.baselineSmoothness(i)   = safeGet(b.mean, 'smoothness');
    S.irpscFailures(i)        = safeGet(a.mean, 'plannerFailures');
    S.baselineFailures(i)     = safeGet(b.mean, 'plannerFailures');
end
end

% =====================================================================
function v = safeGet(s, f)
if isstruct(s) && isfield(s, f)
    v = s.(f);
else
    v = NaN;
end
end
