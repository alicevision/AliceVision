__version__ = "1.0"

import json
import math
import os

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMComparing(desc.Node):
    """
Compare repeated SfM reconstructions of one scene with a reference set of runs.

Each input is one run of the same reconstruction, for example the same graph with different random seeds. For every
run the node measures the reconstructed views, the landmarks and the reprojection RMSE, and records the camera poses.
Given a reference (the statistics file this node wrote for an earlier set of runs), it compares the two sets:

 - **Views, landmarks, RMSE**: Welch's t-test (unequal variances), two-sided. A difference that is significant and goes
   the wrong way gives a warning, and fails when it is also larger than the metric's tolerance.
 - **Camera poses**: every run is aligned to every reference run by a similarity on the centres of the cameras both
   reconstructed. The residual of the centres (RMS, relative to their spread) and the median rotation difference are
   compared with the same measures between the reference runs themselves, which are the run-to-run noise. Without
   ground truth this says "different", not "worse", so it fails only past an absolute floor as well.

A failed comparison raises an error, which stops the pipeline. Without a reference the node only measures the runs;
its statistics file can then serve as the reference of later comparisons.
"""

    category = "Utils"
    inputs = [
        desc.ListAttribute(
            elementDesc=desc.File(
                name="sfmData",
                label="SfMData",
                description="SfMData file of one run.",
                value="",
            ),
            name="input",
            label="Runs",
            description="SfMData files of the runs to compare, one per run.",
            exposed=True,
        ),
        desc.File(
            name="reference",
            label="Reference Statistics",
            description="Statistics file written by this node for the reference set of runs.\n"
                        "If empty, the runs are only measured.",
            value="",
        ),
        desc.FloatParam(
            name="significanceLevel",
            label="Significance Level",
            description="A difference counts as significant when its two-sided p-value is below this level.",
            value=0.05,
            range=(0.001, 0.5, 0.001),
            advanced=True,
        ),
        desc.FloatParam(
            name="viewsTolerance",
            label="Views Tolerance (%)",
            description="Significant loss of reconstructed views, in percent of the reference mean, that only warns.",
            value=0.0,
            range=(0.0, 100.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="landmarksTolerance",
            label="Landmarks Tolerance (%)",
            description="Significant loss of landmarks, in percent of the reference mean, that only warns.",
            value=0.5,
            range=(0.0, 100.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="rmseTolerance",
            label="RMSE Tolerance (%)",
            description="Significant increase of the reprojection RMSE, in percent of the reference mean, that only warns.",
            value=0.5,
            range=(0.0, 100.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="posesWarningRatio",
            label="Poses Warning Ratio",
            description="Pose differences to the reference runs warn above this multiple of the differences between "
                        "the reference runs themselves.",
            value=1.5,
            range=(1.0, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="posesFailureRatio",
            label="Poses Failure Ratio",
            description="Pose differences to the reference runs fail above this multiple of the differences between "
                        "the reference runs themselves.",
            value=3.0,
            range=(1.0, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="centersFloor",
            label="Camera Centers Floor",
            description="Differences of the camera centres below this fraction of their spread never warn or fail.",
            value=0.001,
            range=(0.0, 0.1, 0.0001),
            advanced=True,
        ),
        desc.FloatParam(
            name="rotationsFloor",
            label="Rotations Floor (Degrees)",
            description="Rotation differences below this angle never warn or fail.",
            value=0.1,
            range=(0.0, 10.0, 0.01),
            advanced=True,
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
        ),
    ]

    outputs = [
        desc.File(
            name="output",
            label="Statistics",
            description="Measurements and camera poses of every run (JSON). It can serve as the reference of a later "
                        "comparison.",
            value="{nodeCacheFolder}/statistics.json",
        ),
        desc.File(
            name="report",
            label="Report",
            description="The comparison as Markdown tables.",
            value="{nodeCacheFolder}/report.md",
        ),
    ]

    def processChunk(self, chunk):
        from pyalicevision import sfm as avsfm
        from pyalicevision import sfmData as avsfmdata
        from pyalicevision import sfmDataIO as avsfmdataio

        chunk.logManager.start(chunk.node.verboseLevel.value)
        try:
            node = chunk.node
            runs = []
            for attribute in node.input.value.values():
                path = attribute.value
                data = avsfmdata.SfMData()
                if not avsfmdataio.load(data, path, avsfmdataio.ALL):
                    raise RuntimeError(f"Cannot open the SfMData file '{path}'.")
                rmse = avsfm.RMSE(data)
                if rmse < 0:  # no observation at all
                    raise RuntimeError(f"The run '{path}' has no landmarks: its reconstruction failed.")
                run = _measure(data, rmse, path)
                chunk.logger.info(f"{path}: {run['views']} views, {run['landmarks']} landmarks, RMSE {run['rmse']:.6g}")
                runs.append(run)
            if len(runs) < 2:
                raise RuntimeError("At least two runs are needed to measure their spread.")

            statistics = {"version": 1, "runs": runs}
            with open(node.output.value, "w") as f:
                json.dump(statistics, f, indent=1)

            if not node.reference.value:
                lines = _measurementsReport(runs)
                overall = "MEASURED"
            else:
                with open(node.reference.value) as f:
                    reference = json.load(f)
                if len(reference.get("runs", [])) < 2:
                    raise RuntimeError(f"The reference '{node.reference.value}' has fewer than two runs.")
                lines, overall = _comparisonReport(reference["runs"], runs, node)

            with open(node.report.value, "w") as f:
                f.write("\n".join(lines) + "\n")
            for line in lines:
                chunk.logger.info(line)

            if overall == "FAIL":
                chunk.logger.error("The runs are worse than the reference (see the report).")
                raise RuntimeError("SfM comparison failed.")
        finally:
            chunk.logManager.end()


# ---------------------------------------------------------------------------------------------------------- measures
def _measure(data, rmse, path):
    """Reconstructed views, landmarks, RMSE and, per reconstructed view, the pose (world-to-camera rotation and centre)."""
    poses = {}
    for viewId in data.getValidViews():
        # getPose returns a CameraPose by value and getTransform a reference into it: keep the pose alive meanwhile
        cameraPose = data.getPose(data.getView(viewId))
        transform = cameraPose.getTransform()
        rotation = transform.rotation()
        center = transform.center()
        poses[str(viewId)] = {
            "rotation": [float(rotation[i][j]) for i in range(3) for j in range(3)],
            "center": [float(center[i]) for i in range(3)],
        }
    return {"input": os.path.basename(os.path.dirname(path)) + "/" + os.path.basename(path),
            "views": len(poses), "landmarks": len(data.getLandmarks()), "rmse": float(rmse), "poses": poses}


# ---------------------------------------------------------------------------------------------------------- statistics
def _incompleteBetaFraction(a, b, x):
    """The continued fraction 1 / (1 + d1 / (1 + d2 / (1 + ...))) of the regularized incomplete beta function
    (DLMF 8.17.22), by the modified Lentz method: numerators 1, d1, d2, ..., every partial denominator 1."""
    tiny = 1e-300
    f, c, d = tiny, tiny, 0.0
    for k in range(1000):
        m = k // 2
        if k == 0:
            numerator = 1.0
        elif k % 2:   # d_(2m+1)
            numerator = -(a + m) * (a + b + m) * x / ((a + 2 * m) * (a + 2 * m + 1))
        else:         # d_(2m)
            numerator = m * (b - m) * x / ((a + 2 * m - 1) * (a + 2 * m))
        d = 1.0 + numerator * d
        d = 1.0 / (d if abs(d) > tiny else tiny)
        c = 1.0 + numerator / c
        c = c if abs(c) > tiny else tiny
        delta = c * d
        f *= delta
        if k > 0 and abs(delta - 1.0) < 1e-15:
            break
    return f


def _regularizedIncompleteBeta(a, b, x):
    """I_x(a, b) for 0 <= x <= 1."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    logFront = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1.0 - x)
    if x < (a + 1.0) / (a + b + 2.0):
        return math.exp(logFront) * _incompleteBetaFraction(a, b, x) / a
    return 1.0 - math.exp(logFront) * _incompleteBetaFraction(b, a, 1.0 - x) / b


def _twoSidedP(t, df):
    """Two-sided p-value of Student's t with df degrees of freedom."""
    return _regularizedIncompleteBeta(df / 2.0, 0.5, df / (df + t * t))


def _criticalT(df, alpha):
    """The |t| whose two-sided p-value is alpha (bisection)."""
    lo, hi = 0.0, 1000.0
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if _twoSidedP(mid, df) > alpha:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _meanVar(values):
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / (n - 1) if n > 1 else 0.0
    return mean, var


def _welch(reference, candidate):
    """Welch's t-test, two-sided, on candidate minus reference."""
    nr, nc = len(reference), len(candidate)
    mr, vr = _meanVar(reference)
    mc, vc = _meanVar(candidate)
    se2 = vr / nr + vc / nc
    if se2 == 0.0:
        same = mr == mc
        return {"diff": mc - mr, "t": 0.0 if same else math.copysign(math.inf, mc - mr), "df": nr + nc - 2.0,
                "p": 1.0 if same else 0.0, "se": 0.0}
    t = (mc - mr) / math.sqrt(se2)
    df = se2 ** 2 / ((vr / nr) ** 2 / max(nr - 1, 1) + (vc / nc) ** 2 / max(nc - 1, 1))
    return {"diff": mc - mr, "t": t, "df": df, "p": _twoSidedP(t, df), "se": math.sqrt(se2)}


# ---------------------------------------------------------------------------------------------------------- geometry
def _alignment(source, target):
    """Similarity from source to target on the centres of the views both runs reconstructed (Umeyama 1991); the
    centres' residual RMS relative to their spread, and the median rotation difference in degrees."""
    import numpy as np
    common = sorted(set(source) & set(target))
    if len(common) < 3:
        return None
    P = np.array([source[k]["center"] for k in common])
    Q = np.array([target[k]["center"] for k in common])
    X, Y = P - P.mean(0), Q - Q.mean(0)
    U, D, Vt = np.linalg.svd(Y.T @ X / len(common))
    S = np.diag([1.0, 1.0, -1.0 if np.linalg.det(U) * np.linalg.det(Vt) < 0 else 1.0])
    R = U @ S @ Vt
    scale = float(np.trace(np.diag(D) @ S) / (X ** 2).sum(1).mean())
    residual = Y - scale * (X @ R.T)
    spread = math.sqrt(float((Y ** 2).sum(1).mean()))
    if spread == 0.0:
        return None
    angles = []
    for k in common:
        # x_cam = Rc (x_world - C); under x' = s R x + t the world-to-camera rotation becomes Rc R^T
        Rs = np.array(source[k]["rotation"]).reshape(3, 3) @ R.T
        Rt = np.array(target[k]["rotation"]).reshape(3, 3)
        c = (np.trace(Rt.T @ Rs) - 1.0) / 2.0
        angles.append(math.degrees(math.acos(max(-1.0, min(1.0, float(c))))))
    return {"centers": math.sqrt(float((residual ** 2).sum(1).mean())) / spread,
            "rotations": float(np.median(angles))}


def _median(values):
    s = sorted(values)
    n = len(s)
    return 0.0 if n == 0 else (s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2]))


# ---------------------------------------------------------------------------------------------------------- reports
METRICS = [  # name, label, the direction that is worse (-1: lower), tolerance attribute
    ("views", "reconstructed views", -1, "viewsTolerance"),
    ("landmarks", "landmarks", -1, "landmarksTolerance"),
    ("rmse", "reprojection RMSE", +1, "rmseTolerance"),
]


def _measurementsReport(runs):
    lines = [f"# SfM runs: {len(runs)}", "", "| metric | mean | standard deviation | min | max |", "|---|---|---|---|---|"]
    for name, label, _, _ in METRICS:
        values = [r[name] for r in runs]
        mean, var = _meanVar(values)
        lines.append(f"| {label} | {mean:.6g} | {math.sqrt(var):.3g} | {min(values):.6g} | {max(values):.6g} |")
    pairs = [_alignment(a["poses"], b["poses"]) for i, a in enumerate(runs) for b in runs[i + 1:]]
    pairs = [p for p in pairs if p]
    lines += ["", f"Between the runs ({len(pairs)} pairs): camera centres {_median([p['centers'] for p in pairs]):.3g} of "
              f"their spread, rotations {_median([p['rotations'] for p in pairs]):.3g} degrees (medians)."]
    return lines


def _comparisonReport(reference, runs, node):
    alpha = node.significanceLevel.value
    lines = [f"# SfM runs ({len(runs)}) against the reference ({len(reference)} runs)", "",
             "| metric | reference mean (sd) | runs mean (sd) | difference | t | p | resolution | verdict |",
             "|---|---|---|---|---|---|---|---|"]
    verdicts = []
    for name, label, worse, toleranceName in METRICS:
        ref = [float(r[name]) for r in reference]
        cand = [float(r[name]) for r in runs]
        w = _welch(ref, cand)
        mr, vr = _meanVar(ref)
        mc, vc = _meanVar(cand)
        pct = 100.0 * w["diff"] / mr if mr else 0.0
        tolerance = getattr(node, toleranceName).value
        verdict = "PASS"
        if w["p"] < alpha:
            if w["diff"] * worse > 0:
                verdict = "FAIL" if abs(pct) > tolerance else "WARN"
            else:
                verdict = "PASS (better)"
        # the smallest difference this comparison could have called significant: a PASS means none larger than this
        resolution = _criticalT(w["df"], alpha) * w["se"] if w["se"] > 0 else 0.0
        verdicts.append(verdict)
        lines.append(f"| {label} | {mr:.6g} ({math.sqrt(vr):.3g}) | {mc:.6g} ({math.sqrt(vc):.3g}) | "
                     f"{w['diff']:+.4g} ({pct:+.3f} %) | {w['t']:+.2f} | {w['p']:.4f} | "
                     f"{resolution:.3g} ({100.0 * resolution / mr if mr else 0.0:.3f} %) | {verdict} (tolerance {tolerance:g} %) |")

    within = [_alignment(a["poses"], b["poses"]) for i, a in enumerate(reference) for b in reference[i + 1:]]
    cross = [_alignment(r["poses"], b["poses"]) for r in runs for b in reference]
    within = [p for p in within if p]
    cross = [p for p in cross if p]
    if not within or not cross:
        # poses are matched by view id: different images, or view ids derived from another location (CameraInit's
        # default hashes the folder of images without a serial number), leave nothing to compare
        raise RuntimeError("The runs and the reference share fewer than three reconstructed views: they were not made "
                           "from the same images with the same view ids.")
    lines += ["", "| poses | between reference runs (median) | runs against reference (median) | ratio | floor | verdict |",
              "|---|---|---|---|---|---|"]
    for key, label, floor in (("centers", "camera centres, RMS / spread", node.centersFloor.value),
                              ("rotations", "rotations, degrees", node.rotationsFloor.value)):
        mw = _median([p[key] for p in within])
        mc = _median([p[key] for p in cross])
        ratio = mc / mw if mw > 0 else (math.inf if mc > 0 else 1.0)
        verdict = "PASS"
        if mc > floor:
            if ratio > node.posesFailureRatio.value:
                verdict = "FAIL"
            elif ratio > node.posesWarningRatio.value:
                verdict = "WARN"
        verdicts.append(verdict)
        lines.append(f"| {label} | {mw:.3g} | {mc:.3g} | {ratio:.2f} | {floor:g} | {verdict} |")

    overall = "FAIL" if "FAIL" in verdicts else ("WARN" if "WARN" in verdicts else "PASS")
    lines += ["", f"Overall: {overall} (significance level {alpha:g}; poses warn above {node.posesWarningRatio.value:g}x and "
              f"fail above {node.posesFailureRatio.value:g}x the differences between the reference runs, past the floor)"]
    return lines, overall
