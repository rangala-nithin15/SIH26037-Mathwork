function results = runAllExperiments(seeds, saveDir)
%RUNALLEXPERIMENTS Run the full evaluation: comparison plus ablation.
%
%   COMPONENT STATUS: REAL
%
%   results = RUNALLEXPERIMENTS(seeds, saveDir) runs the complete evaluation
%   used for the SIH submission: baseline versus IR-PSC on all five
%   scenarios, then the five-way ablation study.
%
%   This is the one command that produces every number in the report.
%
%   ============================================================
%   BEFORE YOU RUN THIS
%   ============================================================
%   Nothing in this project has ever been executed. Run the unit tests
%   first, fix whatever fails, and only then run the experiments. Numbers
%   produced by code whose tests have not passed are not worth reporting.
%
%       runAllTests('unit')
%       runAllTests('all')
%       runAllExperiments(1:10)
%
%   ============================================================
%   RUNTIME
%   ============================================================
%   The comparison runs 5 scenarios x nSeeds x 2 planners. The ablation runs
%   5 scenarios x nSeeds x 6 variants. With 10 seeds that is 400 full
%   simulations, which will take a while. Start with 3 seeds to check it
%   works end to end, then scale up.
%
%   Inputs:
%       seeds   - (optional) vector of integer seeds. Default 1:5.
%                 More seeds means tighter confidence, not better results.
%       saveDir - (optional) folder for .mat outputs. Default 'results'.
%
%   Outputs:
%       results - struct with fields .comparison and .ablation
%
%   Example:
%       setupPaths;
%       r = runAllExperiments(1:10);
%
%   Requires: base MATLAB only.
%
%   See also RUNCOMPARISON, RUNABLATION, PRINTCOMPARISON, RUNALLTESTS.

if nargin < 1 || isempty(seeds),   seeds = 1:5; end
if nargin < 2 || isempty(saveDir)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    saveDir = fullfile(projectRoot, 'results');
end

if exist(saveDir, 'dir') ~= 7
    mkdir(saveDir);
end

scenarios = {'village','urban','highway','market','cattle'};

fprintf('\n');
fprintf('=====================================================================\n');
fprintf('  SIH_Indian_AV FULL EVALUATION\n');
fprintf('  Scenarios: %s\n', strjoin(scenarios, ', '));
fprintf('  Seeds:     %s\n', mat2str(seeds));
fprintf('  Total runs: %d comparison + %d ablation = %d\n', ...
        numel(scenarios)*numel(seeds)*2, ...
        numel(scenarios)*numel(seeds)*6, ...
        numel(scenarios)*numel(seeds)*8);
fprintf('=====================================================================\n\n');

tStart = tic;

% --- 1. Baseline versus IR-PSC ------------------------------------------
fprintf('--- PART 1 of 2: baseline vs IR-PSC ---\n\n');
opts = struct('verbose', true, 'saveDir', saveDir);
results.comparison = runComparison(scenarios, seeds, opts);

fprintf('\n');
printComparison(results.comparison);

% --- 2. Ablation study ---------------------------------------------------
fprintf('--- PART 2 of 2: IR-PSC ablation study ---\n\n');
results.ablation = runAblation(scenarios, seeds, opts);

printAblation(results.ablation);

% --- Save ----------------------------------------------------------------
stamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
fname = fullfile(saveDir, sprintf('full_evaluation_%s.mat', stamp));
save(fname, 'results');

elapsed = toc(tStart);
fprintf('\n');
fprintf('=====================================================================\n');
fprintf('  Evaluation complete in %.1f minutes.\n', elapsed/60);
fprintf('  Saved to: %s\n', fname);
fprintf('\n');
fprintf('  Every number above was computed from an executed run. Report\n');
fprintf('  them as they are, including any result where IR-PSC does not\n');
fprintf('  win -- a negative finding is a legitimate finding, and a\n');
fprintf('  comparison that can only come out one way is not a comparison.\n');
fprintf('=====================================================================\n\n');
end

% =====================================================================
function printAblation(ab)
%PRINTABLATION Print the ablation deltas relative to the full system.
S = ab.summary;

fprintf('\n');
fprintf('=====================================================================\n');
fprintf('  ABLATION STUDY: change relative to the full IR-PSC system\n');
fprintf('  Generated: %s\n', ab.timestamp);
fprintf('=====================================================================\n\n');
fprintf('  A NEGATIVE clearance delta means the ablated variant was LESS\n');
fprintf('  safe, i.e. that component was genuinely contributing.\n\n');

fprintf('%-18s', 'Variant');
for si = 1:numel(S.scenarios)
    fprintf('%12s', S.scenarios{si});
end
fprintf('\n%s\n', repmat('-', 1, 18 + 12*numel(S.scenarios)));

fprintf('  d(worst clearance), metres\n');
for vi = 2:numel(S.variants)      % skip 'full', which is the reference
    fprintf('%-18s', S.variants{vi});
    for si = 1:numel(S.scenarios)
        fprintf('%12.3f', S.deltaMinClearance(vi,si));
    end
    fprintf('\n');
end

fprintf('\n  d(completion rate)\n');
for vi = 2:numel(S.variants)
    fprintf('%-18s', S.variants{vi});
    for si = 1:numel(S.scenarios)
        fprintf('%12.3f', S.deltaCompletion(vi,si));
    end
    fprintf('\n');
end

fprintf('\n  d(average speed), m/s\n');
for vi = 2:numel(S.variants)
    fprintf('%-18s', S.variants{vi});
    for si = 1:numel(S.scenarios)
        fprintf('%12.3f', S.deltaAvgSpeed(vi,si));
    end
    fprintf('\n');
end

fprintf('\n');
fprintf('  If a variant shows deltas near zero everywhere, that component\n');
fprintf('  is not earning its complexity in these scenarios. Report that\n');
fprintf('  honestly rather than assuming it must be helping.\n');
fprintf('=====================================================================\n\n');
end
