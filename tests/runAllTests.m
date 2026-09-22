function results = runAllTests(mode)
%RUNALLTESTS Run the project's test suites and print an honest summary.
%
%   COMPONENT STATUS: REAL
%
%   results = RUNALLTESTS()        runs the unit tests only (fast)
%   results = RUNALLTESTS('all')   runs unit and integration tests
%   results = RUNALLTESTS('unit')  same as the default
%   results = RUNALLTESTS('integration') runs integration tests only
%
%   ============================================================
%   THE HEADLINE FACT ABOUT THIS TEST SUITE
%   ============================================================
%   As of packaging, NONE of these tests has ever been executed. MATLAB was
%   not installed on the machine where this project was written, so running
%   them was impossible. Every test file says so in its own header.
%
%   Do not report any test in this project as passing until you have run it
%   and seen it pass. Report the number you actually get, including
%   failures. A test suite with known failures that are documented is
%   honest engineering; a suite claimed to pass without evidence is not.
%
%   Expect some failures on first run. This is normal for a codebase of this
%   size written without an interpreter available to check it. Fix them,
%   then record the real numbers in PROJECT_STATUS.md.
%
%   Inputs:
%       mode - (optional) 'unit' (default), 'integration', or 'all'
%
%   Outputs:
%       results - matlab.unittest.TestResult array
%
%   Example:
%       setupPaths;
%       results = runAllTests('all');
%
%   Requires: base MATLAB only (matlab.unittest is part of base MATLAB).
%
%   See also TESTGEOMETRY, TESTCORRIDOR, TESTPREDICTIONANDRISK,
%            TESTPLANNERCORE, TESTDECISIONANDVEHICLE, TESTCLOSEDLOOP.

if nargin < 1 || isempty(mode)
    mode = 'unit';
end
mode = lower(char(mode));

unitSuites = {'testGeometry', 'testCorridor', 'testPredictionAndRisk', ...
              'testPlannerCore', 'testDecisionAndVehicle'};
integSuites = {'testClosedLoop', 'testBehaviour'};   % testBehaviour added in Phase 2

switch mode
    case 'unit'
        suites = unitSuites;
    case 'integration'
        suites = integSuites;
    case 'all'
        suites = [unitSuites, integSuites];
    otherwise
        error('runAllTests:unknownMode', ...
              'Unknown mode "%s". Use unit | integration | all.', mode);
end

fprintf('\n');
fprintf('=====================================================================\n');
fprintf('  SIH_Indian_AV test run (%s)\n', mode);
fprintf('=====================================================================\n');

results = [];
for i = 1:numel(suites)
    fprintf('\n--- %s ---\n', suites{i});
    try
        r = runtests(suites{i});
        results = [results, r]; %#ok<AGROW>
    catch ME
        fprintf(2, 'Suite %s could not run: %s\n', suites{i}, ME.message);
    end
end

% --- Summary -------------------------------------------------------------
if isempty(results)
    fprintf('\nNo tests ran. Check that setupPaths has been called.\n');
    return;
end

nPassed   = sum([results.Passed]);
nFailed   = sum([results.Failed]);
nIncomplete = sum([results.Incomplete]);
nTotal    = numel(results);

fprintf('\n');
fprintf('=====================================================================\n');
fprintf('  RESULT: %d of %d passed, %d failed, %d incomplete\n', ...
        nPassed, nTotal, nFailed, nIncomplete);
fprintf('  Total time: %.2f s\n', sum([results.Duration]));
fprintf('=====================================================================\n');

if nFailed > 0
    fprintf('\nFAILED TESTS:\n');
    for i = 1:numel(results)
        if results(i).Failed
            fprintf('  - %s\n', results(i).Name);
        end
    end
    fprintf(['\nThese are real failures. Fix them, and record the real ' ...
             'numbers in\nPROJECT_STATUS.md. Do not report this suite as ' ...
             'passing until it does.\n']);
end
fprintf('\n');
end
