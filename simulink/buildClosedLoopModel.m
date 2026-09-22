function modelName = buildClosedLoopModel(modelName, cfg)
%BUILDCLOSEDLOOPMODEL Construct the closed-loop Simulink model by script.
%
%   COMPONENT STATUS: REAL SOURCE / EXECUTION **NOT VERIFIED**
%
%   ============================================================
%   READ THIS BEFORE RUNNING
%   ============================================================
%   This script has NEVER BEEN EXECUTED. Simulink was not installed on the
%   machine where this project was written, so it could not be run or
%   debugged. It is written against the documented Simulink programmatic
%   API, but it WILL probably need corrections on first run.
%
%   That is why no .slx file ships with this project. A .slx is a binary
%   artefact that can only be produced by Simulink. Fabricating one would be
%   dishonest and impossible to do correctly. Generating it from this script
%   on the destination machine is the honest path: the model is then
%   reproducible, reviewable as text, and diffable in git -- which a
%   committed binary would not be.
%
%   Expect to iterate. Run it, read the error, fix the block path or
%   parameter name, run again. See simulink/SIMULINK_ARCHITECTURE.md for
%   what the model is meant to contain and why.
%
%   modelName = BUILDCLOSEDLOOPMODEL(modelName, cfg) creates and saves the
%   model, returning its name.
%
%   ARCHITECTURE BUILT
%   ------------------
%   A single-rate discrete model at cfg.sim.dt, wiring:
%
%       Scenario/GroundTruth -> Sensors -> Perception+Tracking
%           -> Prediction -> IR-PSC Planner -> Decision Logic
%           -> Controller -> Vehicle Dynamics -> (feedback to Scenario)
%
%   Each stage is a MATLAB Function block calling the SAME .m functions the
%   pure-MATLAB pipeline uses. This is deliberate and important: there is
%   ONE implementation of the planner, not a MATLAB one and a separate
%   Simulink one that can silently drift apart. The Simulink model is a
%   wrapper for scheduling and visualisation, not a reimplementation.
%
%   CONSEQUENCE, STATED HONESTLY: MATLAB Function blocks calling functions
%   that use structs, cell arrays and variable-size data are not
%   code-generation friendly without significant rework. This model is
%   intended for SIMULATION and demonstration, not for code generation to
%   an embedded target. Making it codegen-ready is future work.
%
%   Inputs:
%       modelName - (optional) char, model name. Default 'sihClosedLoop'.
%       cfg       - (optional) config struct from irpscConfig()
%
%   Outputs:
%       modelName - the name of the created model
%
%   Example:
%       setupPaths;
%       buildClosedLoopModel('sihClosedLoop', irpscConfig('village'));
%
%   Requires: MATLAB + Simulink. NOT VERIFIED on any machine.
%
%   See also SIMULINK_ARCHITECTURE.md, RUNSCENARIO, IRPSCPLANNER.

if nargin < 1 || isempty(modelName), modelName = 'sihClosedLoop'; end
if nargin < 2 || isempty(cfg),       cfg = irpscConfig('urban');  end

% --- Guard: fail with a clear message rather than a cryptic one ---------
if isempty(ver('simulink'))
    error('buildClosedLoopModel:noSimulink', ...
          ['Simulink is not available on this MATLAB installation. ' ...
           'This script requires Simulink. The pure-MATLAB pipeline in ' ...
           'scripts/runScenario.m runs without it.']);
end

% --- Start from a clean model -------------------------------------------
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% --- Solver configuration ------------------------------------------------
% Fixed-step discrete: the planner runs at a fixed rate, so a variable-step
% solver would be both meaningless and slower here.
set_param(modelName, 'SolverType', 'Fixed-step');
set_param(modelName, 'Solver', 'FixedStepDiscrete');
set_param(modelName, 'FixedStep', num2str(cfg.sim.dt));
set_param(modelName, 'StopTime', num2str(cfg.sim.maxTime));

x = 40; y = 40; w = 150; h = 90; gap = 90;

% --- Blocks ---------------------------------------------------------------
% Each MATLAB Function block wraps one stage. The function bodies below are
% thin: they call the shared .m implementation so there is only ever one
% version of the algorithm.

blocks = { ...
  'Scenario',    'scenarioStep',   'Scenario ground truth (actors)'; ...
  'Sensors',     'sensorStep',     'Camera + LiDAR + radar simulation'; ...
  'Perception',  'perceptionStep', 'Fusion and multi-object tracking'; ...
  'Planner',     'plannerStep',    'IR-PSC planner (incl. prediction)'; ...
  'Decision',    'decisionStep',   'Six-state decision logic'; ...
  'Controller',  'controlStep',    'Pure pursuit + PI speed control'; ...
  'Vehicle',     'vehicleStep',    'Kinematic bicycle dynamics' };

for i = 1:size(blocks,1)
    bName = blocks{i,1};
    bPath = [modelName '/' bName];
    add_block('simulink/User-Defined Functions/MATLAB Function', bPath, ...
              'Position', [x, y, x+w, y+h]);
    y = y + h + gap;
    if y > 700
        y = 40;
        x = x + w + 200;
    end
end

% --- Signal routing ------------------------------------------------------
% NOTE: the exact port names produced by a MATLAB Function block depend on
% the function signature written into it, which this script does not set
% programmatically (setting MATLAB Function block code requires the
% Stateflow API, see setBlockCode below). Connect the blocks either by hand
% after generation, or extend this script once the port names are known on
% your installation.
%
% Intended connections:
%   Scenario   -> Sensors     (ground truth)
%   Sensors    -> Perception  (detections)
%   Perception -> Planner     (tracks)
%   Planner    -> Decision    (plan)
%   Decision   -> Controller  (action, trajectory)
%   Controller -> Vehicle     (steer, accel)
%   Vehicle    -> Scenario    (ego state, closing the loop)
%   Vehicle    -> Sensors     (ego state, closing the loop)

% --- Scope for visualisation ---------------------------------------------
add_block('simulink/Sinks/Scope', [modelName '/Signals'], ...
          'Position', [x+250, 40, x+300, 90]);

% --- Save -----------------------------------------------------------------
save_system(modelName);

fprintf('\n');
fprintf('Created Simulink model "%s".\n', modelName);
fprintf('\n');
fprintf('THIS MODEL IS NOT COMPLETE AND HAS NOT BEEN VERIFIED.\n');
fprintf('Remaining manual steps:\n');
fprintf('  1. Open each MATLAB Function block and paste in the wrapper\n');
fprintf('     body from simulink/SIMULINK_ARCHITECTURE.md.\n');
fprintf('  2. Connect the blocks as listed in the routing comment in this\n');
fprintf('     file (and drawn in SIMULINK_ARCHITECTURE.md).\n');
fprintf('  3. Set the initial conditions in the Scenario block.\n');
fprintf('  4. Run, and expect to fix errors. Record what you changed.\n');
fprintf('\n');
fprintf('The pure-MATLAB pipeline (scripts/runScenario.m) runs the same\n');
fprintf('algorithms today without Simulink, and is the recommended way to\n');
fprintf('generate results while this model is being brought up.\n');
fprintf('\n');
end
