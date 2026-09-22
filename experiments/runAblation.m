function results = runAblation(scenarioNames, seeds, opts)
%RUNABLATION Measure the contribution of each IR-PSC component.
%
%   COMPONENT STATUS: REAL
%
%   results = RUNABLATION(scenarioNames, seeds, opts) runs the full IR-PSC
%   planner and then five reduced variants, each with exactly one component
%   disabled, under identical conditions.
%
%   ============================================================
%   WHY ABLATION IS THE STRONGEST EVIDENCE AVAILABLE HERE
%   ============================================================
%   Beating a baseline shows the system as a whole works. It does not show
%   WHICH part is doing the work, and it is entirely possible for a
%   component to contribute nothing while the overall system still wins.
%
%   Ablation answers that directly. If disabling prediction barely changes
%   the result, then prediction is not earning its complexity in these
%   scenarios, and that should be reported rather than quietly assumed away.
%
%   THE FIVE ABLATIONS
%   ------------------
%     no_prediction   usePrediction = false. Road users are treated as
%                     frozen where they are. Isolates the value of
%                     predicting future occupancy at all.
%     no_uncertainty  useUncertainty = false. Predictions become point
%                     estimates with negligible sigma, so hazard ellipses
%                     no longer widen with uncertainty. Isolates the value
%                     of representing how unsure we are.
%     no_confidence   useConfidenceAware = false. Confidence is forced to
%                     1.0, so the system never becomes conservative on the
%                     basis of poor inputs. Isolates the value of the
%                     degradation behaviour.
%     no_deformation  useDeformation = false. The trajectory follows the
%                     corridor centre without bending around hazards.
%                     Isolates the value of local deformation.
%     no_smoothing    useTemporalSmoothing = false. No frame-to-frame
%                     blending of corridor or offset profile. Isolates the
%                     value of temporal consistency; expect this to show up
%                     mainly in the smoothness metric.
%
%   Every variant uses the same seeds as the full system, so differences
%   cannot be attributed to different noise.
%
%   Inputs:
%       scenarioNames - cellstr of scenarios, or [] for all five
%       seeds         - vector of integer seeds, or [] for 1:5
%       opts          - (optional) struct passed to runScenario, plus
%                       .verbose and .saveDir
%
%   Outputs:
%       results - struct with fields:
%           .variants   cellstr of variant names, 'full' first
%           .scenarios  cellstr
%           .seeds      the seeds used
%           .metrics    nVariants x nScenarios cell of aggregate structs
%           .summary    struct comparing each variant against 'full'
%           .timestamp  when the study was run
%
%   Example:
%       r = runAblation({'market'}, 1:10, struct('verbose',true));
%
%   Requires: base MATLAB only.
%
%   See also RUNCOMPARISON, IRPSCCONFIG, RUNSCENARIO.

if nargin < 1 || isempty(scenarioNames)
    scenarioNames = {'village','urban','highway','market','cattle'};
end
if nargin < 2 || isempty(seeds)
    seeds = 1:5;
end
if nargin < 3, opts = struct(); end

verbose = isfield(opts,'verbose') && opts.verbose;

variants = {'full','no_prediction','no_uncertainty','no_confidence', ...
            'no_deformation','no_smoothing'};

nV = numel(variants);
nS = numel(scenarioNames);
nR = numel(seeds);

metrics = cell(nV, nS);

for vi = 1:nV
    for si = 1:nS
        perRun = cell(1, nR);
        for ri = 1:nR
            cfg = irpscConfig(profileFor(scenarioNames{si}));
            cfg = applyAblation(cfg, variants{vi});

            runOpts = opts;
            runOpts.seed = seeds(ri);

            [~, M] = runScenario(scenarioNames{si}, @irpscPlanner, cfg, runOpts);
            perRun{ri} = M;
        end
        metrics{vi, si} = aggregateMetrics(perRun, ...
                            sprintf('%s / %s', variants{vi}, scenarioNames{si}));
        if verbose
            a = metrics{vi, si};
            fprintf('%-16s %-10s | worst clear %.2f m | completion %.2f | avg speed %.2f m/s\n', ...
                    variants{vi}, scenarioNames{si}, ...
                    a.worstMinClearance, a.completionRate, ...
                    safeGet(a.mean,'averageSpeed'));
        end
    end
end

results.variants  = variants;
results.scenarios = scenarioNames;
results.seeds     = seeds;
results.metrics   = metrics;
results.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
results.summary   = buildAblationSummary(variants, scenarioNames, metrics);

if isfield(opts,'saveDir') && ~isempty(opts.saveDir)
    if exist(opts.saveDir, 'dir') ~= 7
        mkdir(opts.saveDir);
    end
    fname = fullfile(opts.saveDir, ...
                     sprintf('ablation_%s.mat', datestr(now,'yyyymmdd_HHMMSS'))); %#ok<TNOW1,DATST>
    save(fname, 'results');
    fprintf('Saved ablation study to %s\n', fname);
end
end

% =====================================================================
function cfg = applyAblation(cfg, variant)
%APPLYABLATION Disable exactly one component.
switch variant
    case 'full'
        % everything enabled
    case 'no_prediction'
        cfg.ablation.usePrediction = false;
    case 'no_uncertainty'
        cfg.ablation.useUncertainty = false;
    case 'no_confidence'
        cfg.ablation.useConfidenceAware = false;
    case 'no_deformation'
        cfg.ablation.useDeformation = false;
    case 'no_smoothing'
        cfg.ablation.useTemporalSmoothing = false;
    otherwise
        error('runAblation:unknownVariant', 'Unknown variant "%s".', variant);
end
end

% =====================================================================
function p = profileFor(scenarioName)
switch lower(scenarioName)
    case 'village',                     p = 'village';
    case {'urban','urban_intersection'},p = 'urban';
    case {'highway','highway_merge'},   p = 'highway';
    case 'market',                      p = 'market';
    case {'cattle','cattle_crossing'},  p = 'cattle';
    otherwise,                          p = 'urban';
end
end

% =====================================================================
function S = buildAblationSummary(variants, scenarioNames, metrics)
%BUILDABLATIONSUMMARY Difference of each variant from 'full', per scenario.
%   A NEGATIVE clearance delta means the ablated variant was less safe than
%   the full system, i.e. that component was contributing.
S.variants  = variants;
S.scenarios = scenarioNames;
S.deltaMinClearance = nan(numel(variants), numel(scenarioNames));
S.deltaCompletion   = nan(numel(variants), numel(scenarioNames));
S.deltaAvgSpeed     = nan(numel(variants), numel(scenarioNames));
S.deltaSmoothness   = nan(numel(variants), numel(scenarioNames));

for si = 1:numel(scenarioNames)
    base = metrics{1, si};        % 'full' is always index 1
    for vi = 1:numel(variants)
        a = metrics{vi, si};
        S.deltaMinClearance(vi,si) = a.worstMinClearance - base.worstMinClearance;
        S.deltaCompletion(vi,si)   = a.completionRate - base.completionRate;
        S.deltaAvgSpeed(vi,si)     = safeGet(a.mean,'averageSpeed') - ...
                                     safeGet(base.mean,'averageSpeed');
        S.deltaSmoothness(vi,si)   = safeGet(a.mean,'smoothness') - ...
                                     safeGet(base.mean,'smoothness');
    end
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
