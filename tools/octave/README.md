# GNU Octave compatibility layer (development checks only)

**MATLAB is the target platform. This folder exists so the project can be executed and tested on a machine without MATLAB.** It was used in Phase 2 because MATLAB was not installed on the development laptop.

## What is here

| File | Replaces (MATLAB) | Notes |
|---|---|---|
| `RandStream.m` | `RandStream` | Seeded MRG32k3a generator with `rand`/`randn`/`randi` methods. **Numbers differ from MATLAB's Mersenne Twister**, so an Octave run is not numerically identical to a MATLAB run with the same seed. Reproducible within Octave. |
| `functiontests.m` | `functiontests` | Returns the local-function handles |
| `verify*.m` | `matlab.unittest` qualifications | Throw on failure; one failure ends that test |
| `runOctaveTests.m` | `runtests` | Runs function-based test files, prints PASS/FAIL |
| `setupOctave.m` | — | Runs `setupPaths`, then adds this folder |

## Usage

```
octave --no-gui
>> addpath('tools/octave'); setupOctave
>> r = runOctaveTests();                      % all suites
>> r = runOctaveTests({'testBehaviour'});     % one suite
```

## Rules

- **Never add this folder to the MATLAB path.** `setupPaths.m` does not add it; `setupOctave` refuses to run in MATLAB.
- **Report Octave results as Octave results.** They are evidence the code runs and behaves as tested in Octave, not a MATLAB validation.
- Octave has no `VideoWriter`; `runDemo` writes PNG frames instead. Off-screen rendering needs the `qt` graphics toolkit (`octave.exe`, not `octave-cli.exe`).
