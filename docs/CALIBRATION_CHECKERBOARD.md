# Camera calibration (checkerboard)

Computes the camera **intrinsics** of the CSI camera: the matrix `K` (focal
length `fx,fy` and optical center `cx,cy`) and the lens **distortion**
coefficients. With those, `cv2.undistort` straightens the lines bent by the lens,
and all the downstream geometry (line follower, intersection detection) is
correct.

> Do this **first**. The illumination calibration
> ([CALIBRATION_ILLUMINATION.md](CALIBRATION_ILLUMINATION.md)) comes **after**,
> because it uses already-undistorted images.

The previous `config/camera_params.npz` came from a **different camera**, so it
must be redone with this CSI in its mounted pose.

---

## Step 0: focus (before calibrating)

Set the lens **focus** first and **do not touch it afterwards** (changing focus
slightly alters the intrinsics). The focus assistant measures it live over H264 —
point at a textured target (the board or printed text) at the working distance
and **turn the lens to MAXIMIZE** the number:

```bash
scripts/run_focus_assist_jetson.sh
```

The bar/number show sharpness vs the peak; when it says **"AT PEAK"** it is in
focus. There is no universal "good" number: you just maximize over a fixed scene.
Once focus is fixed, continue with the board capture.

## What you need

- The board printed and **glued flat onto something rigid** (your cardboard
  works). If it bends, the calibration comes out wrong.
- Your board: **6x8 squares -> `5x7` inner-corner pattern** (the crosses where 4
  squares meet, not counting the border). The tool also auto-detects in case you
  count it in another orientation.
- Good **even** light, no reflections/glare on the paper.

---

## How to take the samples (this is what makes it robust)

Capture is **auto-guided**: you watch the H264 stream on the laptop and move the
board following the on-screen hints. The tool **only saves** photos that are
sharp, still, and that **add a new pose**. You need variety on 3 axes:

1. **Position in the frame** — bring the board to the **4 corners and the center**
   of the frame (cover the 3x3 grid). That calibrates the lens edges well, where
   it distorts the most.
2. **Distance** — take **near, medium and far** (the board filling a lot, medium
   and little of the frame). But it must **always be fully visible**.
3. **Tilt** — tilt it **+/-30-45 deg** up, down, left and right, and **rotate it
   in-plane**. Never all photos flat and head-on: that gives a poor calibration.

Rules of thumb:

- **Move slowly and pause** for a moment at each pose (if there is motion, the
  photo is blurry and the tool rejects it: it will say "blurry").
- **Ideal: 20-30 varied views.** That is the sweet spot (OpenCV/MATLAB
  consensus). From 30 to 50 you gain very little; beyond ~50 it no longer
  improves. **Quality > quantity:** 15 excellent views are worth more than 60
  mediocre ones. The default is `TARGET=30`.
- **You do not need symmetry/parallelism.** The computation is a _global_
  optimization; tilting left at one edge and not "mirroring" it on the right
  **does not matter**. What matters is: touch the **whole frame** (the 4 corners
  and edges) and include tilts in **both directions** overall — not that they
  match the position.
- Avoid **reflections** and hard shadows on the board.
- The board should fill a good part of the frame, but **whole**, never clipped.

The overlay tells you what is missing: _"move the board farther away"_, _"move
the board closer"_, _"front yaw <= -16 deg (bring the left edge closer)"_, and the
progress `18/30  dist 3/3  pose 4/6`. It also shows `front pitch`, `front yaw` and
`roll2d` in degrees so you know how much you are tilting. `front pitch/yaw` near 0
means the board is head-on to the camera.

---

## Steps

```bash
# 1) Auto-guided capture (H264 preview on the laptop). Wheels-up does not apply
#    (it does not move the robot). Move the board until you reach the target.
scripts/run_checkerboard_capture_jetson.sh
#    Default 30 captures. Optional variables: TARGET=40  PATTERN=5x7
#    It stops on its own when it reaches the target, or Ctrl+C to stop.
#    Images are fetched to ./calibration_images/ on the laptop.

# 2) Compute the calibration (runs on the Jetson; OpenCV is not on the laptop).
scripts/run_calibrate_camera_jetson.sh
#    Brings back config/camera_params.npz and config/undistorted_preview.jpg
```

### How to read the result

Step 2 prints:

```
RMS reprojection error: 0.34 px  (640x480)
Verdict: EXCELLENT
```

- **RMS < 0.5 px** -> excellent.
- **0.5-1.0 px** -> acceptable.
- **> 1.0 px** -> recapture with **more variety** (especially tilts and frame
  corners). The tool already drops the worst images (outliers) automatically
  before reporting.

Open **`config/undistorted_preview.jpg`** (left original | right undistorted):
the straight lines of the scene should look **straight** in the corrected
version.

### Save

```bash
git add config/camera_params.npz
git commit -m "Recalibrate CSI camera intrinsics"
# (a later sync re-pushes it to the Jetson; the raw images stay in
#  calibration_images/, ignored by git)
```

---

## Technical details

- Capture and calibration run at **640x480**, the **same** resolution as the
  runtime, so `K` is valid without rescaling (`line_follower`, `sign_detector`,
  etc. use 640x480).
- Detection with `cv2.findChessboardCornersSB` (robust to blur/light) with a
  fallback to the classic detector + `cornerSubPix`. Computation with
  `cv2.calibrateCamera`.
- The physical square size (`SQUARE_MM`) **does not affect `K`** or the
  distortion; it is only stored as metadata. You can ignore it.
- Code: `tools/calib_capture_checkerboard.py` (capture) and
  `tools/calibrate_camera.py` (computation).
