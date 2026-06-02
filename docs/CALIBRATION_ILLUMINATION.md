# Illumination calibration (flat-field) - "the reds"

Corrects the **reddish tint** and the **vignetting** (darker corners) of the
camera. The sensor does not respond equally across all pixels or channels, so a
white surface shows up with a **reddish blob** and dark edges. This calibration
measures that pattern over a **uniform white banner/sheet** and builds a per-pixel,
per-channel **gain map** that flattens it. Result: even whites, no tint, and far
more stable color/line masks.

> Do this **AFTER** the camera calibration
> ([CALIBRATION_CHECKERBOARD.md](CALIBRATION_CHECKERBOARD.md)), because the map is
> measured on **already-undistorted** images (the same ones the runtime will see).

---

## What you need

- A **white, matte, uniform** surface that fills the whole frame: the white
  banner, a large sheet, or a clean white wall. Matte, not glossy (gloss causes
  specular reflections).
- The **same light** you will have on the track. If you calibrate under different
  light, it is useless.
- **Diffuse and even** light, no shadows or direct spotlights.

---

## How to take the samples (robustness)

It is **auto-guided**, but now it **does not start capturing until you press
Enter**. First you position the banner watching the H264; when it looks good you
press Enter, remove hands/shadows, wait a few seconds and the tool collects
several good frames and **averages** them (lowers noise) before computing. It only
accepts frames that pass the quality gate; the overlay tells you what to fix:

1. **Fill the frame with the banner** - no background, no banner edges, no
   objects. Only white.
2. **No shadows** - neither yours nor the robot's on the banner. If being fully
   head-on casts a shadow, use a **slight angle** toward the banner. For
   flat-field it matters more that the whole frame sees uniform white than being
   perfectly perpendicular. If there is a dark area, it will say *"shadow
   detected: light it evenly"*.
3. **No specular highlights** - if a light reflects, change the angle. A slight
   tilt usually helps. It will say *"specular highlight: change the angle"*.
4. **Correct exposure** - neither too dark nor blown out. It will say *"too
   dark"* or *"too bright"*. Aim for an even gray-white, not blown-out white.
5. **Still** - keep the camera steady while it collects the frames.
6. **Press Enter only when you are ready** - after Enter there is a short wait to
   remove your hand and keep your shadow out of the average.

When it collects the target frames (25 by default) it computes, saves and stops on
its own.

---

## Steps

```bash
# Point at the white banner filling the frame.
scripts/run_illumination_calibrator_jetson.sh
#    Watch the H264, position the banner, press Enter when ready.
#    Optional: FRAMES=30
#    AUTO_START=1 restores the previous behavior.
#    Collects good frames, computes and saves.
#    Brings back config/illumination_flatfield.npz and config/illumination_preview.jpg
```

### How to verify

The tool prints a residual report:

```
std BGR before: 31.4 28.9 40.2
std BGR after :  6.1  5.8  6.4
mean BGR before: 150.2 158.7 196.1     <- high R = reddish tint
mean BGR after : 171.0 171.2 171.4     <- even channels = tint removed
```

- The **std after** should drop noticeably (a "flatter" image).
- The **mean BGR after** should be **even** across channels (red stops
  dominating = the reddish blob is gone).

Open **`config/illumination_preview.jpg`** (left raw average | right corrected):
the right side should look **uniform** white, without the reddish blob or dark
corners.

### Save

```bash
git add config/illumination_flatfield.npz
git commit -m "Recalibrate illumination flat-field"
```

---

## Technical details

- Works at **640x480** and, if `config/camera_params.npz` exists, applies
  `undistort` before measuring (consistent with the runtime).
- Gain = `channel_mean / blurred_reference`, clipped to `[0.25, 4.0]`; ~25 good
  frames are averaged to reduce noise.
- The runtime applies it via `apply_illumination_gain` (in
  `puzzlebot_ros/perception/camera.py`). The `.npz` format (key `gain`) is
  unchanged, so all existing consumers use it without changes.
- Code: `tools/illumination_calibrator.py`.
