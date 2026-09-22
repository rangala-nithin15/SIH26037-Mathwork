# 3D visualisation — architecture, settings, measured performance

**Date:** 2026-09-21 · **Branch:** `phase2-demo`

**Execution environment:** GNU Octave 11.3.0 on Windows 11. MATLAB is still not installed, so **nothing here has been run in MATLAB**. Every number below was measured in Octave on the development laptop. During the measurements another program on the same laptop was using about six CPU cores, so absolute timings are pessimistic and varied between runs; comparisons that matter were made back-to-back under the same conditions.

The renderer is schematic MATLAB/Octave 3D graphics (patches, lines, text). It is not RoadRunner, not Unreal Engine and not photorealistic.

---

## 1. Layers and the rule between them

```
SIMULATION          runScenario: actors, sensors, fusion, tracking, dynamics, referee
    ↓
PLANNER / IR-PSC    irpscPlanner + decisionLogic (unchanged by this work)
    ↓
VISUALISATION STATE the frame struct: ego, truth, tracks, detections, plan, action, log
    ↓               (replay: rebuilt from the recorded log; interpFrame blends poses)
RENDERER            updateDemoViewer  (+ viewerEvents, applyVisualMode)
    ↓
DISPLAY             figure window, or frameRecorder → video file
```

The renderer only **reads** the frame. It returns nothing to the simulation except "keep going / stop" (window closed or **q** pressed). Planner, prediction, risk, corridor, safety, decision, controller, dynamics, scenarios, metrics and `config/irpscConfig.m` were not modified; `git diff` shows changes only in the files listed in section 7.

Evidence that simulation behaviour is unchanged: the demo scenario, seed 1, run through `runDemo` with the final renderer in the loop, reproduces the Phase 2 result exactly — goal reached at 76.9 s, 0 collisions, worst clearance 0.53 m, average speed 4.04 m/s, 9 safe-stop episodes, 0 emergency-braking steps, 8 planner failure cycles, 2 pothole wheel entries (pothole 4 at 3.5 and 3.9 m/s).

## 2. Where to change things — `visualization/viewer3d/vizConfig.m`

| Setting | Field | Section of the file |
|---|---|---|
| Display FPS target (15 / 20 / 30 / 45 / 60) | `vc.displayFps` (default 30) | Timing, mode and camera |
| Visual mode | `vc.visualMode` or `vizConfig('planning')` | Timing, mode and camera / Visual modes |
| Camera | `vc.camera` (`chase` / `overview` / `top`) | Timing, mode and camera |
| Colours | `vc.color.*` | Semantic palette |
| Transparency | `vc.alpha.*` | Transparency |
| Layer update rates | `vc.rate.*` | Layer update rates |
| What is shown (HUD, performance read-out, prediction, risk, corridor, trail, …) | `vc.show.*` | Layer visibility |

```matlab
vc = vizConfig('planning', struct('displayFps', 60, 'camera', 'overview'));
replayDemo('results/demo_run.mat', struct('viz', vc))
```

## 3. Visual language

Every colour keeps one meaning everywhere:

| Colour | Meaning |
|---|---|
| teal / green | space the vehicle may use — the drivable corridor |
| cyan | what the vehicle intends to do — the planned trajectory |
| amber | caution — following, yielding, waiting before a blocked passage |
| red | stop / high risk — safe-stop trajectory, stop wall, high-risk cells, STOP badge |
| orange-red, translucent | predicted occupancy of other road users (1 / 2 / 3 s, 1-σ) |
| yellow → orange → red | risk cells, low → medium → high (the DP's own risk table) |
| violet | pothole traversal cost |
| muted neutrals | the world: asphalt, buildings, trees, parked vehicles, stalls |

Visual modes: **full** (everything), **planning** (corridor, trajectory, hazards, prediction; no sensor plumbing), **sensing** (detections, tracks, predictions; no risk/corridor fill), **minimal** (road, ego, road users, trajectory). Keys **f p s m** switch at run time; **1 2 3** switch camera.

The right-hand panel answers, top to bottom: which state (CRUISE / SLOW / AVOID / STOP / RESUME), how risky (LOW / MEDIUM / HIGH, from the planner's own thresholds), how confident, TTC, speed, detections, potholes, and what the planner is doing (SAFE / AVOIDING / FOLLOWING / SLOWING / WAITING / STOPPED). The decision log below it explains behaviour in steps. Verbatim from the exported 30 FPS video of the full demo as the cow enters the road:

```
57.2s [PREDICT]  Path conflict predicted in 2.3 s
57.6s [PREDICT]  Predicted occupancy on path (risk 1.00)
59.0s [DETECT]   Cattle ahead, 10 m
59.8s [DECISION] AVOID
59.8s [DECISION] STOP
```

Every line restates something the simulation reported (tracker, pothole tracker, planner behaviour flags and status, decision-logic state changes); `viewerEvents.m` derives nothing else. Two consequences worth knowing: `[DETECT]` fires when the tracker first reports a *confirmed track id* ahead within 45 m, so a re-initialised track (as for the cow here) is announced when its new id appears, after the prediction that was already driving the behaviour; and the log is built from the frames actually drawn, so a replay started part-way through can show slightly different times for the first events.

## 4. Why the old renderer was slow — measured

Baseline (original renderer, off-screen, first 20 s of the demo): `updateDemoViewer` took 98 ms per frame (median 95, max 238). The profiler put **76 % of render time inside `set`**: 74 property writes per frame at about 1 ms each. Nothing was being recreated per frame; the cost was the number of writes.

What changed:

1. **Dirty checking** (`setCached`): a property is written only if its value changed (`isequaln`, because cleared layers hold NaN and `isequal(NaN,NaN)` is false). Writes per frame fell from 74 to about 9–14.
2. **Layer tiering** (`vc.rate`): risk, prediction, pothole cost, tracks, detections, HUD, mini-map and time series update every 2nd–6th frame; ego, road users, trail and camera every frame.
3. **Interpolated "tween" frames**: between two simulation states only the ego, road users, trail and camera move (`interpFrame`, `frame.tween`).
4. **Less data per write**: risk cells merged two offsets per displayed cell (display only), trail and mini-map path thinned, static world merged into a few patches.
5. **Camera smoothing in simulated time** instead of per frame, so camera motion does not change with the display rate.

## 5. The real ceiling on screen in GNU Octave — measured

Once the property writes were fixed, **painting the window** became the limit. Octave's Qt/OpenGL figure repaints the whole canvas (all four axes and every text object) whenever anything moves, and the chase camera moves every frame. Measured on screen:

| Experiment (one frame = camera move + `drawnow`) | ms per frame |
|---|---|
| full scene | 52–55 |
| all 70 patches hidden | 24 (window, axes and text alone) |
| terrain hidden / 59 small patches hidden | 49 / 47 |
| quads → triangles; lighting off | no change |
| 9 writes as one multi-handle `set` vs 9 separate `set`s | 55.8 vs 55.4 (no gain) |

A/B on screen, same 501 recorded states (t = 20–45 s), full redraw every state, runs interleaved old/new/old/new:

| Renderer | mean ms | median ms | max ms |
|---|---|---|---|
| original | 258 / 494 | 249 / 306 | 1008 / 3884 |
| redesigned | 227 / 219 | 211 / 207 | 723 / 601 |

The redesign is faster and far steadier on screen, but the paint floor dominates, so the on-screen gain is modest.

Two rendering defects were found and fixed along the way. The goal marker was not drawn in some renders: its row/column data made Octave skip it with "x/y/zdata must have the same dimensions". This was inherited from the original viewer. Translucent overlays rendered opaque: in Octave a lit patch ignores `FaceAlpha` when the axes has a light, so every translucent overlay now uses `FaceLighting 'none'`.

## 6. Benchmarks (GNU Octave 11.3.0, this laptop, under the background load described above)

**Replay on screen, t = 20–45 s of the recorded demo, paced to the wall clock** (measured before the transparency and layout fixes; see section 6b for the final code):

| Target FPS | Achieved FPS | Mean frame ms | Max frame ms | Mean render ms | Frames dropped to keep real time |
|---|---|---|---|---|---|
| 15 | 7.4 | 135.9 | 355.1 | 124.8 | 192 |
| 30 | 7.3 | 136.7 | 284.5 | 126.2 | 568 |
| 60 | 7.5 | 134.5 | 293.6 | 128.8 | 1323 |
| unpaced | 8.0 | 125.0 | 2102.4 | 116.9 | — |

So **on screen in Octave, about 7–8 FPS is what this laptop achieves, whatever the target.** Replay keeps simulated time running at wall-clock speed by dropping display frames and reports how many it dropped; it never claims a rate it did not draw. 30 and 60 FPS are **not** achieved on screen in Octave here.

**Live `runDemo`, full demo, off-screen, final code:** 770 full + 769 interpolated frames; mean render 63.5 ms; mean frame (simulation + render) 246.7 ms → 4.1 display FPS; 407 s wall clock for 76.9 s simulated. An earlier identical run under heavier background load took 604 s. In a live run the planner, not the renderer, sets the pace.

**Smooth 30 FPS: exported video.** `replayDemo(..., struct('videoFile', 'results/irpsc_demo_30fps.mp4'))` draws every 1/30 s frame off-screen and assembles an H.264 MP4 with the ffmpeg bundled with GNU Octave (`frameRecorder.m`). The video plays at a true 30 FPS in PowerPoint, a browser or VLC, independent of how fast the laptop draws. Export cost and verification are in section 6b.

### 6b. Final-code measurements

All with the finished code, demo scenario, seed 1, on this laptop under the background load described at the top.

**Replay on screen, t = 20–40 s, paced to the wall clock:**

| Target FPS | Achieved FPS | Mean frame ms | Max frame ms | Mean render ms | Frames dropped | Wall clock for 20 s shown |
|---|---|---|---|---|---|---|
| 15 | 5.5 | 181.4 | 472.5 | 143.0 | 190 | 20.1 s |
| 30 | 5.6 | 178.6 | 408.7 | 136.0 | 490 | 20.1 s |
| 60 | 6.6 | 151.2 | 386.1 | 114.6 | 1069 | 20.0 s |
| unpaced | 8.4 | 120.0 | 359.3 | 99.6 | 0 | 71.8 s |

Paced replay stays in real time (20 s shown in 20 s) by dropping frames. It does **not** reach 15, 30 or 60 FPS on screen in Octave here; 5.5–8.4 FPS is what was drawn.

**Live `runDemo` on screen, first 20 s of the demo:** 201 full + 199 interpolated frames, mean render 116.7 ms, mean frame (simulation + render) 558.9 ms → 1.8 FPS; 148.0 s wall clock for 20.0 s simulated.

**Live `runDemo` off screen, full demo** (the validation run): 4.1 FPS, 63.5 ms mean render, 407 s for 76.9 s. Simulation results identical to Phase 2 (section 1).

**30 FPS MP4 export:** `results/irpsc_demo_30fps.mp4`, H.264, 30 fps, 76.93 s, 2 308 frames (1 539 full redraws + 769 interpolated), 2000 × 1148, fully decodable (checked with ffmpeg). Two exports were made: the first (3 547 s, 59 min) had the performance read-out burned into every frame, so recordings now hide it; the second (2 496 s, 42 min; mean viewer update 67 ms, the rest is writing each PNG) is the current file. **The current file still shows the ghost-label bug described in section 8 and must be re-exported after that fix.** `results/` is git-ignored, so the video is local only.

**Tests:** `runOctaveTests` — 117 of 118 pass, unchanged; the one failure is the pre-existing `testFrenetRoundTrip`.

**Not measured:** anything in MATLAB. MATLAB renders on the GPU and repaints only at `drawnow`, so the dirty-checking and tiering should matter there and 30 FPS on screen is plausible, but this is untested and must not be claimed.

## 7. Files

| File | Change |
|---|---|
| `visualization/viewer3d/vizConfig.m` | **new** — every display setting in one place |
| `visualization/viewer3d/updateDemoViewer.m` | rewritten update path: dirty checking, tiered layers, tween frames, semantic colours, new HUD, performance read-out |
| `visualization/viewer3d/createDemoViewer.m` | semantic palette, merged static geometry, layer groups for visual modes, new HUD layout, transparency fix, goal-marker fix |
| `visualization/viewer3d/actorModel3D.m` | cleaner low-poly models; ego with roof stripe and sensor pod; palette colours |
| `visualization/viewer3d/applyVisualMode.m` | **new** — full / planning / sensing / minimal |
| `visualization/viewer3d/interpFrame.m` | **new** — visual interpolation between two recorded states |
| `visualization/viewer3d/viewerEvents.m` | **new** — decision log ([DETECT] [PREDICT] [PLAN] [RISK] [DECISION]) |
| `visualization/viewer3d/frameRecorder.m` | **new** — VideoWriter in MATLAB; PNG frames + bundled ffmpeg → MP4 in Octave |
| `scripts/runDemo.m` | draws every state (full + tween), performance measurement and summary, visual-mode option, shared recorder at 1/dt FPS |
| `scripts/replayDemo.m` | presentation mode: FPS target, interpolation, real-time pacing with honest frame dropping, MP4 export, performance report |
| `RUN_GUIDE.md`, `JUDGE_DEMO_GUIDE.md` | updated for the new viewer, replay mode and colours |

## 8. Known issues (open — not yet fixed)

1. **Ghost labels / actors (visible bug).** `setCached` in `updateDemoViewer.m` stores "show" and "hide" writes for the same object under *different* cache keys (`tlbl%d` / `tlblOff%d`, `plbl%d` / `plblOff%d`, `actor%d` / `actorOff%d`, around lines 97/101, 336/340 and 374–384). The first hide works; after the object is shown again, the second hide matches the stale cached "hidden" value and is skipped, so the label or road-user model stays on screen at its last position. Seen in the exported video: two stale "T118 bicycle" labels while the HUD reports 0 tracks. **Fix:** use one cache key per object for its visibility (hide with the same key as show). The MP4 must be re-exported afterwards (~40–60 min on this laptop).
2. `vc.interpolate` and `vc.maxTweenFrames` in `vizConfig.m` are defined but never read: changing them has no effect. Remove them or wire them up.
3. `vc.cameraLag` comment in `vizConfig.m` says "0 = snaps, 1 = never catches up"; the code uses it as a time constant in seconds (`alpha = 1 - exp(-dt / cameraLag)`).
4. `updateDemoViewer.m` header points to `docs/PHASE2_CHANGES.md` for the performance design; it is this file.
5. `PROJECT_STATUS.md` still quotes "151 s for 76.9 s simulated" for the demo; `runDemo` now draws every simulation state and took 407 s off screen in the final validation run (under background load).
6. `JUDGE_DEMO_GUIDE.md` says `runDemo` shows the behaviour "planned in real time, slower" — it is planned live, much slower than real time on this laptop.
7. Cosmetic: tentative pothole label reads "(1 hits)"; the scenario name is truncated in the HUD; `vc.fpsOptions` is not enforced.
8. **Presentation guidance:** on this laptop, on-screen replay reaches only 5.5–8.4 FPS and a full live `runDemo` takes several minutes, so present the exported MP4 (after issue 1 is fixed), not a live run.
9. **Never run in MATLAB.** All viewer code, including the MATLAB-only paths (`VideoWriter`, `drawnow('limitrate')`), is untested in MATLAB.

