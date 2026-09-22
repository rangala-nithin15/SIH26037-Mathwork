# Perception Notes

**What the perception chain does, what it does not do, and what may never be claimed from it.**

---

## 1. The single most important fact

> **This project performs no object detection.**
>
> No image is processed. No point cloud is segmented. No neural network runs. No detector has been trained, and none has been evaluated.
>
> `simulateDetections.m` takes **ground-truth object positions** from the scenario and corrupts them with geometry and noise.

That is a legitimate and common way to study **planning under imperfect perception**, which is what this project studies. It is not a way to study perception.

### Consequently, these statements are forbidden

- ❌ "Our detector achieves *X*% accuracy"
- ❌ "We detect auto-rickshaws and cattle"
- ❌ "Detection precision / recall / mAP is …"
- ❌ "The perception system was validated"

### These statements are accurate

- ✅ "Perception is simulated geometrically, with realistic failure modes."
- ✅ "The planner is exercised against occlusion, missed detections, false positives and class confusion."
- ✅ "No detector has been trained; replacing the simulated detector with a real one is future work, and the interface is designed for it."

---

## 2. What *is* modelled, and why each matters

| Effect | Why it is in there |
|---|---|
| **Field of view and range gating** | A hazard outside the sensor cone does not exist to the planner |
| **Occlusion** | The one that matters most on Indian roads — a motorcycle emerging from behind a bus is the characteristic surprise |
| **Range-dependent noise** | Distant objects are less precisely located, so early decisions rest on worse data |
| **Distance-dependent missed detections** | Detection probability falls with range |
| **False positives** | Radar clutter in particular; tests that the tracker's M-of-N confirmation prevents phantom braking |
| **Class confusion** | Biased toward similar sizes: a distant auto-rickshaw is called a car, not a bus |

Occlusion is worth dwelling on. It is what makes the urban intersection scenario hard: corner buildings hide cross traffic until late, so a planner that reacts to *current* positions is provably too slow, and prediction has to do real work.

---

## 3. The three sensors and why all three

Complementary failure modes are the entire justification for fusion:

| | Camera | LiDAR | Radar |
|---|---|---|---|
| Range accuracy | poor (0.60 m + 4.5%/m) | **excellent** (0.05 m) | good (0.25 m) |
| Bearing accuracy | **excellent** (0.4°) | excellent (0.25°) | poor (2.0°) |
| Velocity | none | none | **direct Doppler** |
| Class | **yes — the only one** | no | no |
| FOV | 90° | 120° | 45° |
| Max range | 60 m | 80 m | **120 m** |

Inverse-covariance fusion means **LiDAR dominates the position estimate automatically**, because its covariance is smallest — no hand-tuned per-sensor weights anywhere. The camera supplies the class, the radar supplies the velocity.

> **These numbers are plausible, not measured.** They are not from any datasheet and have not been validated against real hardware. They exist to produce realistic *failure modes*, not realistic *performance*.

---

## 4. The ten classes, and the honest gap

`objectClasses.m` defines ten road-user classes. The `.cocoNative` flag records something important:

| Class | Stock COCO detector has it? |
|---|---|
| car, bus, truck, motorcycle, bicycle | ✅ yes |
| pedestrian (COCO "person") | ✅ yes |
| **auto-rickshaw** | ❌ **no** |
| **pushcart** | ❌ **no** |
| **animal / cattle** | ❌ **no** (COCO has "cow", but not Indian road cattle in context) |
| unknown | — |

**Three of the most characteristically Indian road users are not classes any off-the-shelf detector provides.** Detecting them requires a custom annotated dataset — thousands of labelled images of Indian traffic. That work has not been done here, and pretending otherwise would be the easiest and most damaging overclaim available.

The flag exists in the code precisely so nobody can mistake *intent* for *capability*.

---

## 5. Fusion and tracking

### `fuseDetections.m` — SIMPLIFIED

Real algorithm: inverse-covariance weighting, which is the standard optimal linear combination for independent Gaussian measurements.

```
P_fused = inv( Σ inv(P_k) )
x_fused = P_fused · Σ ( inv(P_k) · x_k )
```

Simplifications, stated plainly:
- Association is **greedy nearest-neighbour** under a Mahalanobis gate, not a global assignment.
- Sensor errors are assumed **independent**. In reality they are not entirely — weather affects camera and LiDAR together.

Confidence rises when sensors agree, capped at +0.25. The cap exists because agreement between *correlated* errors is not independent evidence.

### `multiObjectTracker.m` — FALLBACK (F5)

Replaces `trackerGNN` / `trackerJPDA` from Sensor Fusion and Tracking Toolbox.

- Constant-velocity Kalman filter, state `[px vx py vy]`
- Joseph-form covariance update, which stays symmetric positive-definite under round-off where the simpler `(I−KH)P` form does not
- Chi-square gate at 99% (2 dof)
- **M-of-N confirmation:** 3 hits to confirm, 5 misses to delete
- **Coasting** carries a track through short occlusions — this is what lets a road user hidden behind a bus be re-associated on reappearance
- Confidence **decays 15% per coasted frame**, and that decay flows into planner confidence and therefore into behaviour

**Only confirmed tracks reach the planner**, so a single clutter detection cannot cause a brake.

**Known limitation:** greedy assignment is not globally optimal and **can swap two tracks that pass close together** in a dense crowd. JPDA exists precisely for that case. This is acknowledged rather than hidden, and the interface is designed so a toolbox tracker drops in unchanged.

---

## 6. Replacing simulation with real perception

The interface was designed for this. To swap in a real detector, produce a struct array with these fields:

```
.pos        1×2 world position
.vel        1×2 world velocity, [NaN NaN] if unmeasured
.class      char, from objectClasses()
.confidence 0..1
.sensor     char
.posCov     2×2 measurement covariance in the world frame
```

Nothing downstream changes. Suggested order of work:

1. **Camera** — Computer Vision + Deep Learning Toolbox. Needs a custom dataset for auto-rickshaw, pushcart and cattle.
2. **LiDAR** — Lidar Toolbox for clustering and bounding boxes.
3. **Radar** — Automated Driving Toolbox radar models.
4. **Replace the tracker** with `trackerJPDA` once Sensor Fusion and Tracking Toolbox is available.

Getting the covariance right matters more than it looks: fusion weights sensors by it, so a detector that reports an over-confident covariance will dominate the fused estimate and drag it wrong.

---

## 7. Summary of labels

| File | Label | One-line reason |
|---|---|---|
| `sensors/sensorConfig.m` | SIMPLIFIED | Plausible parameters, not measured |
| `perception/detection/simulateDetections.m` | **SIMPLIFIED** | **Geometric model over ground truth — not detection** |
| `perception/fusion/fuseDetections.m` | SIMPLIFIED | Real fusion maths, greedy association |
| `perception/tracking/multiObjectTracker.m` | FALLBACK (F5) | Substitute for toolbox trackers |

All four are **NOT VERIFIED** — none has been executed.
