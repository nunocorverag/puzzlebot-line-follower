"""YOLO traffic-sign detector for the Manchester track.

Detects the painted signs (stop / give_way / workers / turn_left / turn_right /
go_straight) with the trained ``best.pt`` and maps them to driving actions. The
node consumes this to: slow on workers, stop on stop/give_way, and AUTO-decide
the turn at the next cross on the arrow signs.

ROS-free (mirrors lane.py / zebra.py). DEGRADES GRACEFULLY: if ultralytics or the
weights are missing, the detector simply reports "nothing" and the follower keeps
working exactly as before -- so enabling signs can never break line following.

Performance: inference runs only every ``every_n`` frames on the UPPER band of the
frame (signs are above the floor) at a small ``imgsz``, so it does not slow the
30 Hz control loop.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass
class SignParams:
    model_path: str = ""           # path to best.pt
    conf: float = 0.55             # min YOLO confidence
    band_top_pct: int = 0          # inference ROI: vertical band [top, bot] of frame
    band_bot_pct: int = 60         # signs live in the upper part (above the floor)
    imgsz: int = 320               # inference size (small = fast)
    every_n: int = 5               # run inference 1 of every N frames (~6 Hz @30)
    min_box_pct: float = 1.2       # min box area (% of ROI) -> ignore far/tiny signs
    stable_needed: int = 2         # same canonical class this many detections -> commit


# Tolerant mapping: match by substring so small naming differences in best.pt
# (e.g. "turnLeft", "left_arrow", "roadworks", "yield") still resolve.
_ALIASES = (
    # English + the Spanish class names trained in best.pt:
    #   give-way, stop, straight, trabajadores, vuelta-derecha, vuelta-izquierda
    ("izquierda", "turn_left"), ("left", "turn_left"),
    ("derecha", "turn_right"), ("right", "turn_right"),
    ("straight", "go_straight"), ("recto", "go_straight"),
    ("ahead", "go_straight"), ("forward", "go_straight"), ("adelante", "go_straight"),
    ("stop", "stop"), ("alto", "stop"),
    ("trabaj", "workers"), ("work", "workers"), ("men", "workers"), ("obra", "workers"),
    ("give", "give_way"), ("yield", "give_way"), ("ceda", "give_way"),
)
CANONICAL = {"turn_left", "turn_right", "go_straight", "stop", "workers", "give_way"}


@dataclass
class SignResult:
    name: str | None = None        # canonical name or None (best/selected sign)
    conf: float = 0.0
    box: tuple | None = None       # (x1, y1, x2, y2) in full-frame px
    area_pct: float = 0.0          # box area as % of the ROI
    all_detections: list = None    # list of all detected signs [{name, conf, box, area_pct}]


def _canon(raw) -> str | None:
    r = str(raw).lower()
    for key, val in _ALIASES:
        if key in r:
            return val
    return None


def _verify_arrow_direction(frame, box):
    """Verify arrow direction by analyzing the sign's ROI using multiple robust methods.
    
    Returns: tuple (direction, vote_details) where:
        - direction: 'turn_left', 'turn_right', 'go_straight', or None if unclear
        - vote_details: dict with votes from each method for logging
    
    Multi-method strategy:
    1. Find arrow contours and analyze their geometry
    2. Detect arrow tip position (leftmost/rightmost extreme point)
    3. Analyze edge slopes to find diagonal lines
    4. Combine all methods with voting
    """
    import cv2
    import numpy as np
    
    x1, y1, x2, y2 = (int(v) for v in box)
    roi = frame[y1:y2, x1:x2]
    
    if roi.size == 0:
        return None, {'all_votes': [], 'vote_counts': {}, 'winner': None, 'reason': 'empty_roi'}
    
    h, w = roi.shape[:2]
    if w < 20 or h < 20:  # Too small to analyze
        return None, {'all_votes': [], 'vote_counts': {}, 'winner': None, 'reason': 'small_roi'}
    
    # Convert to grayscale
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # Apply multiple preprocessing methods for robustness
    # Method 1: Otsu threshold
    _, binary1 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Method 2: Adaptive threshold (handles varying lighting)
    binary2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY, 11, 2)
    
    # Combine both methods
    binary = cv2.bitwise_or(binary1, binary2)
    
    # Morphological operations to clean up
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    
    # Invert if needed (arrow should be white on black)
    if cv2.countNonZero(binary) < (h * w * 0.3):
        binary = cv2.bitwise_not(binary)
    
    votes = []  # Collect votes from different methods
    
    # ==================== METHOD 1: CONTOUR ANALYSIS ====================
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Find largest contour (likely the arrow)
        largest_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest_contour) > 100:  # Minimum area
            # Find extreme points
            leftmost = tuple(largest_contour[largest_contour[:, :, 0].argmin()][0])
            rightmost = tuple(largest_contour[largest_contour[:, :, 0].argmax()][0])
            topmost = tuple(largest_contour[largest_contour[:, :, 1].argmin()][0])
            bottommost = tuple(largest_contour[largest_contour[:, :, 1].argmax()][0])
            
            # Calculate centroid
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Analyze arrow tip position relative to centroid
                # Left arrow: leftmost point is far from center
                # Right arrow: rightmost point is far from center
                left_dist = abs(leftmost[0] - cx)
                right_dist = abs(rightmost[0] - cx)
                
                if left_dist > right_dist * 1.4:
                    votes.append('turn_left')
                elif right_dist > left_dist * 1.4:
                    votes.append('turn_right')
                
                # Analyze vertical symmetry for straight arrows
                top_dist = abs(topmost[1] - cy)
                bottom_dist = abs(bottommost[1] - cy)
                if top_dist > max(left_dist, right_dist) * 1.2:
                    votes.append('go_straight')
    
    # ==================== METHOD 2: EDGE SLOPE ANALYSIS ====================
    # Detect edges
    edges = cv2.Canny(binary, 50, 150)
    
    # Hough line detection to find diagonal lines (arrow edges)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, 
                            minLineLength=int(min(h, w) * 0.2), maxLineGap=10)
    
    if lines is not None:
        left_slopes = []
        right_slopes = []
        
        for line in lines:
            x1_l, y1_l, x2_l, y2_l = line[0]
            
            # Calculate slope
            dx = x2_l - x1_l
            dy = y2_l - y1_l
            
            if abs(dx) > 5:  # Avoid vertical lines
                slope = dy / dx
                angle = np.arctan(slope) * 180 / np.pi
                
                # Left arrow: positive slope (going up-left or down-right)
                # Right arrow: negative slope (going up-right or down-left)
                if abs(angle) > 20:  # Significant diagonal
                    if angle > 0:
                        left_slopes.append(angle)
                    else:
                        right_slopes.append(angle)
        
        # Vote based on dominant slopes
        if len(left_slopes) > len(right_slopes) * 1.5:
            votes.append('turn_left')
        elif len(right_slopes) > len(left_slopes) * 1.5:
            votes.append('turn_right')
    
    # ==================== METHOD 3: HORIZONTAL MASS DISTRIBUTION ====================
    # Project onto horizontal axis (sum each column)
    h_projection = np.sum(binary, axis=0)
    
    if len(h_projection) > 0:
        # Find center of mass
        total_mass = np.sum(h_projection)
        if total_mass > 0:
            weighted_sum = np.sum(np.arange(len(h_projection)) * h_projection)
            center_of_mass = weighted_sum / total_mass
            center_pos = center_of_mass / w  # Normalize to 0-1
            
            # Split into thirds and analyze
            third = w // 3
            left_mass = np.sum(h_projection[:third])
            center_mass = np.sum(h_projection[third:2*third])
            right_mass = np.sum(h_projection[2*third:])
            
            total = left_mass + center_mass + right_mass
            if total > 0:
                left_ratio = left_mass / total
                right_ratio = right_mass / total
                center_ratio = center_mass / total
                
                # Vote based on mass distribution
                if left_ratio > 0.42 and left_ratio > right_ratio * 1.3:
                    votes.append('turn_left')
                elif right_ratio > 0.42 and right_ratio > left_ratio * 1.3:
                    votes.append('turn_right')
                elif center_ratio > 0.45:
                    votes.append('go_straight')
    
    # ==================== METHOD 4: CURVED ARROW ANALYSIS (ENHANCED) ====================
    # For curved arrows (common in traffic signs), analyze the arc direction
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        
        if len(largest_contour) > 10:
            # Sample points along the contour
            num_points = min(len(largest_contour), 50)
            indices = np.linspace(0, len(largest_contour)-1, num_points, dtype=int)
            sampled_points = largest_contour[indices].reshape(-1, 2)
            
            x_coords = sampled_points[:, 0]
            y_coords = sampled_points[:, 1]
            
            # METHOD 4A: Top vs Bottom shift (original)
            mid_y = h // 2
            top_points = sampled_points[y_coords < mid_y]
            bottom_points = sampled_points[y_coords >= mid_y]
            
            if len(top_points) > 3 and len(bottom_points) > 3:
                top_x_avg = np.mean(top_points[:, 0])
                bottom_x_avg = np.mean(bottom_points[:, 0])
                shift = top_x_avg - bottom_x_avg
                
                if abs(shift) > w * 0.1:  # Reduced threshold from 0.15 to 0.1
                    if shift > 0:
                        votes.append('turn_right')
                        votes.append('turn_right')
                        votes.append('turn_right')  # Triple vote for curved detection
                    else:
                        votes.append('turn_left')
                        votes.append('turn_left')
                        votes.append('turn_left')  # Triple vote for curved detection
            
            # METHOD 4B: Analyze curvature by fitting polynomial
            # Check if the contour curves left or right
            if len(sampled_points) > 5:
                try:
                    # Fit 2nd degree polynomial to the contour
                    z = np.polyfit(y_coords, x_coords, 2)
                    # z[0] is the coefficient of y^2 (curvature)
                    # Positive curvature = curves right, Negative = curves left
                    if abs(z[0]) > 0.001:  # Significant curvature
                        if z[0] > 0:
                            votes.append('turn_right')
                        else:
                            votes.append('turn_left')
                except:
                    pass
            
            # METHOD 4C: Check rightmost/leftmost points in different vertical zones
            # Divide into 3 vertical zones and check horizontal progression
            third_h = h // 3
            top_zone = sampled_points[y_coords < third_h]
            mid_zone = sampled_points[(y_coords >= third_h) & (y_coords < 2*third_h)]
            bot_zone = sampled_points[y_coords >= 2*third_h]
            
            if len(top_zone) > 2 and len(bot_zone) > 2:
                top_max_x = np.max(top_zone[:, 0])
                bot_max_x = np.max(bot_zone[:, 0])
                
                # If top extends more to the right than bottom = right arrow
                if top_max_x > bot_max_x + w * 0.1:
                    votes.append('turn_right')
                elif bot_max_x > top_max_x + w * 0.1:
                    votes.append('turn_left')
    
    # ==================== VOTING: MAJORITY WINS ====================
    vote_details = {
        'all_votes': votes,
        'vote_counts': {},
        'winner': None
    }
    
    if not votes:
        return None, vote_details
    
    # Count votes
    vote_counts = {}
    for vote in votes:
        vote_counts[vote] = vote_counts.get(vote, 0) + 1
    
    vote_details['vote_counts'] = vote_counts
    
    # Only use this verifier as a left/right override when there is a real
    # consensus. Single-vote corrections were flipping signs on noisy frames.
    turn_counts = {k: v for k, v in vote_counts.items()
                   if k in ('turn_left', 'turn_right')}
    if turn_counts:
        ranked = sorted(turn_counts.items(), key=lambda kv: kv[1], reverse=True)
        winner, winner_count = ranked[0]
        runner_up = ranked[1][1] if len(ranked) > 1 else 0
        if winner_count >= 3 and winner_count - runner_up >= 2:
            vote_details['winner'] = winner
            return winner, vote_details
        vote_details['winner'] = None
        vote_details['reason'] = (
            f'weak_turn_vote winner={winner} votes={winner_count} '
            f'margin={winner_count - runner_up}')

    return None, vote_details  # No clear consensus


class SignDetector:
    def __init__(self, params: SignParams, log: Callable[[str], None] = print):
        self.p = params
        self._log = log
        self._model = None
        self._ok = False
        self._names = {}
        self._frame_i = 0
        self._last = SignResult()
        self._stable_name = None
        self._stable_count = 0
        if not params.model_path:
            log("[signs] disabled (no model_path)")
            return
        try:
            from ultralytics import YOLO
            self._model = YOLO(params.model_path)
            self._names = dict(self._model.names)
            self._ok = True
            log(f"[signs] model loaded: {params.model_path} classes={self._names}")
        except Exception as exc:                       # noqa: BLE001 (degrade on any error)
            self._log(f"[signs] DISABLED (could not load model): {exc}")

    @property
    def ok(self) -> bool:
        return self._ok

    def detect(self, frame) -> SignResult:
        """Return the most confident, debounced sign (name None if none).

        Only actually infers every ``every_n`` frames; in between it returns the
        last result so callers can poll every frame cheaply.
        """
        if not self._ok:
            return SignResult()
        self._frame_i += 1
        if self._frame_i % max(1, self.p.every_n) != 0:
            return self._last

        h, w = frame.shape[:2]
        y0 = max(0, int(h * self.p.band_top_pct / 100.0))
        y1 = min(h, int(h * self.p.band_bot_pct / 100.0))
        roi = frame[y0:y1, :]
        if roi.size == 0:
            self._last = SignResult()
            return self._last
        try:
            preds = self._model.predict(roi, imgsz=self.p.imgsz,
                                        conf=self.p.conf, verbose=False)
        except Exception as exc:                       # noqa: BLE001
            self._log(f"[signs] predict failed: {exc}", )
            self._last = SignResult()
            return self._last

        roi_area = float(roi.shape[0] * roi.shape[1]) or 1.0
        all_signs = []
        for r in preds:
            for b in getattr(r, "boxes", []):
                cf = float(b.conf[0])
                name = _canon(self._names.get(int(b.cls[0]), int(b.cls[0])))
                if name is None:
                    continue
                x1, yb1, x2, yb2 = (float(v) for v in b.xyxy[0])
                area_pct = 100.0 * ((x2 - x1) * (yb2 - yb1)) / roi_area
                if area_pct < self.p.min_box_pct:
                    continue
                
                # SMART VERIFICATION: Only for turn_left/turn_right (curved arrows)
                # go_straight is a different sign (vertical arrow) and should not be verified
                # The model only confuses left/right, not straight
                verified_name = name
                original_name = name  # Keep original for display
                was_corrected = False
                vote_details = None
                
                if name in ('turn_left', 'turn_right'):
                    box_full = (x1, yb1 + y0, x2, yb2 + y0)
                    try:
                        detected_direction, vote_details = _verify_arrow_direction(frame, box_full)
                    except Exception as exc:  # noqa: BLE001 - verification is advisory only
                        self._log(f"[signs] arrow verification failed: {exc}")
                        detected_direction = None
                        vote_details = {'all_votes': [], 'vote_counts': {}, 'winner': None, 'error': str(exc)}
                    # Only accept turn_left or turn_right from verification, ignore go_straight
                    if detected_direction in ('turn_left', 'turn_right'):
                        # Use verified direction instead of model prediction
                        if detected_direction != name:
                            self._log(f"[signs] Direction corrected: {name} -> {detected_direction} "
                                    f"(votes: {vote_details['vote_counts']})")
                            was_corrected = True
                        verified_name = detected_direction
                
                all_signs.append({
                    'name': verified_name,
                    'original_name': original_name if was_corrected else None,
                    'conf': cf,
                    'box': (x1, yb1 + y0, x2, yb2 + y0),
                    'area_pct': area_pct,
                    'score': cf * (area_pct / 100.0),  # Combined score: conf * normalized_area
                    'vote_details': vote_details  # Include verification votes for logging
                })
        
        # Choose best sign: prioritize by combined score (confidence * area)
        # Larger + more confident signs win
        best = SignResult(all_detections=all_signs if all_signs else None)
        if all_signs:
            # Sort by score (descending)
            all_signs.sort(key=lambda s: s['score'], reverse=True)
            top = all_signs[0]
            best.name = top['name']
            best.conf = top['conf']
            best.box = top['box']
            best.area_pct = top['area_pct']

        # Debounce: only surface a sign after it is seen stable_needed times.
        if best.name is not None and best.name == self._stable_name:
            self._stable_count += 1
        else:
            self._stable_name = best.name
            self._stable_count = 1 if best.name is not None else 0
        
        result = best if self._stable_count >= self.p.stable_needed else SignResult(all_detections=all_signs if all_signs else None)
        self._last = result
        return result


def draw_sign_overlay(frame, result: SignResult):
    """Draw ALL detected signs with priority ranking. Selected sign in GREEN, others in YELLOW."""
    import cv2
    
    # Draw all detections if available
    if result.all_detections:
        for i, sign in enumerate(result.all_detections):
            x1, y1, x2, y2 = (int(v) for v in sign['box'])
            # Selected sign (rank 1) = GREEN, others = YELLOW
            is_selected = (result.name == sign['name'] and 
                          result.box == sign['box'])
            color = (0, 255, 0) if is_selected else (0, 255, 255)
            thickness = 3 if is_selected else 2
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            
            # Label: show corrected name only
            rank_marker = "★" if is_selected else f"#{i+1}"
            label = f"{rank_marker} {sign['name']} c:{sign['conf']:.2f} a:{sign['area_pct']:.1f}%"
            
            # Background for text
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(frame, (x1, y1 - th - 4), (x1 + tw, y1), color, -1)
            cv2.putText(frame, label, (x1, y1 - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    
    # Fallback: draw single detection if no all_detections
    elif result.name is not None and result.box is not None:
        x1, y1, x2, y2 = (int(v) for v in result.box)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{result.name} {result.conf:.2f}", (x1, max(12, y1 - 6)),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return frame
