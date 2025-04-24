# gtsam_plotly_modular_v2.py
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import plotly.graph_objects as go
from tqdm.notebook import tqdm

import gtsam

# --- Core Ellipse Calculations ---


def ellipse_path(
    cx: float, cy: float, sizex: float, sizey: float, angle: float, N: int = 60
) -> str:
    """Generates SVG path string for a rotated ellipse."""
    angle_rad = np.radians(angle)
    t = np.linspace(0, 2 * np.pi, N)
    x_unit = (sizex / 2) * np.cos(t)
    y_unit = (sizey / 2) * np.sin(t)
    x_rot = cx + x_unit * np.cos(angle_rad) - y_unit * np.sin(angle_rad)
    y_rot = cy + x_unit * np.sin(angle_rad) + y_unit * np.cos(angle_rad)
    path = (
        f"M {x_rot[0]},{y_rot[0]} "
        + " ".join(f"L{x_},{y_}" for x_, y_ in zip(x_rot[1:], y_rot[1:]))
        + " Z"
    )
    return path


def gtsam_cov_to_plotly_ellipse(
    cov_matrix: np.ndarray, scale: float = 2.0
) -> Tuple[float, float, float]:
    """Calculates ellipse angle (deg), width, height from 2x2 covariance."""
    cov = cov_matrix[:2, :2] + np.eye(2) * 1e-9  # Ensure positive definite
    try:
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvals = np.maximum(eigvals, 1e-9)  # Ensure positive eigenvalues
    except np.linalg.LinAlgError:
        return 0, 0.1 * scale, 0.1 * scale  # Default on failure

    width = 2 * scale * np.sqrt(eigvals[1])  # Major axis (largest eigenvalue)
    height = 2 * scale * np.sqrt(eigvals[0])  # Minor axis (smallest eigenvalue)
    angle_rad = np.arctan2(
        eigvecs[1, 1], eigvecs[0, 1]
    )  # Angle of major axis eigenvector
    angle_deg = np.degrees(angle_rad)
    return angle_deg, width, height


# --- Plotly Element Generators ---


def create_gt_landmarks_trace(landmarks_gt_array: np.ndarray) -> Optional[go.Scatter]:
    """Creates scatter trace for ground truth landmarks."""
    if landmarks_gt_array is None or landmarks_gt_array.size == 0:
        return None
    return go.Scatter(
        x=landmarks_gt_array[0, :],
        y=landmarks_gt_array[1, :],
        mode="markers",
        marker=dict(color="black", size=8, symbol="star"),
        name="Landmarks GT",
    )


def create_gt_path_trace(poses_gt: List[gtsam.Pose2]) -> Optional[go.Scatter]:
    """Creates line trace for ground truth path."""
    if not poses_gt:
        return None
    gt_path_x = [p.x() for p in poses_gt]
    gt_path_y = [p.y() for p in poses_gt]
    return go.Scatter(
        x=gt_path_x,
        y=gt_path_y,
        mode="lines",
        line=dict(color="gray", width=1, dash="dash"),
        name="Path GT",
    )


def create_est_path_trace(
    est_path_x: List[float], est_path_y: List[float]
) -> go.Scatter:
    """Creates scatter/line trace for the estimated path up to current step."""
    return go.Scatter(
        x=est_path_x,
        y=est_path_y,
        mode="lines+markers",
        line=dict(color="red", width=2),
        marker=dict(size=4, color="red"),
        name="Path Est",  # This name applies to the trace in the specific frame
    )


def create_est_landmarks_trace(
    est_landmarks_x: List[float], est_landmarks_y: List[float]
) -> Optional[go.Scatter]:
    """Creates scatter trace for currently estimated landmarks."""
    if not est_landmarks_x:
        return None
    return go.Scatter(
        x=est_landmarks_x,
        y=est_landmarks_y,
        mode="markers",
        marker=dict(color="blue", size=6, symbol="x"),
        name="Landmarks Est",  # Applies to landmarks in the specific frame
    )


def _create_ellipse_shape_dict(
    cx, cy, angle, width, height, fillcolor, line_color, name_suffix
) -> Dict[str, Any]:
    """Helper to create the dictionary for a Plotly ellipse shape."""
    return dict(
        type="path",
        path=ellipse_path(cx=cx, cy=cy, sizex=width, sizey=height, angle=angle),
        xref="x",
        yref="y",
        fillcolor=fillcolor,
        line_color=line_color,
        # name=f"{name_suffix} Cov", # Name isn't really used by Plotly for shapes
    )


def create_pose_ellipse_shape(
    pose_mean_xy: np.ndarray, pose_cov: np.ndarray, k: int, scale: float
) -> Dict[str, Any]:
    """Creates shape dictionary for a pose covariance ellipse."""
    angle, width, height = gtsam_cov_to_plotly_ellipse(pose_cov, scale)
    return _create_ellipse_shape_dict(
        cx=pose_mean_xy[0],
        cy=pose_mean_xy[1],
        angle=angle,
        width=width,
        height=height,
        fillcolor="rgba(255,0,255,0.2)",
        line_color="rgba(255,0,255,0.5)",
        name_suffix=f"Pose {k}",
    )


def create_landmark_ellipse_shape(
    lm_mean_xy: np.ndarray, lm_cov: np.ndarray, lm_index: int, scale: float
) -> Dict[str, Any]:
    """Creates shape dictionary for a landmark covariance ellipse."""
    angle, width, height = gtsam_cov_to_plotly_ellipse(lm_cov, scale)
    return _create_ellipse_shape_dict(
        cx=lm_mean_xy[0],
        cy=lm_mean_xy[1],
        angle=angle,
        width=width,
        height=height,
        fillcolor="rgba(0,0,255,0.1)",
        line_color="rgba(0,0,255,0.3)",
        name_suffix=f"LM {lm_index}",
    )


# --- Frame Content Generation ---


def generate_frame_content(
    k: int,
    step_results: gtsam.Values,
    step_marginals: Optional[gtsam.Marginals],
    X: Callable[[int], int],
    L: Callable[[int], int],
    max_landmark_index: int,  # Need to know the potential range of landmarks
    ellipse_scale: float = 2.0,
    verbose: bool = False,
) -> Tuple[List[go.Scatter], List[Dict[str, Any]]]:
    """Generates all dynamic traces and shapes for a single animation frame `k`."""
    frame_traces: List[go.Scatter] = []
    frame_shapes: List[Dict[str, Any]] = []

    # 1. Gather Estimated Path Data
    est_path_x = []
    est_path_y = []
    for i in range(k + 1):
        pose_key = X(i)
        if step_results.exists(pose_key):
            pose = step_results.atPose2(pose_key)
            est_path_x.append(pose.x())
            est_path_y.append(pose.y())
    frame_traces.append(create_est_path_trace(est_path_x, est_path_y))

    # 2. Gather Estimated Landmark Data
    est_landmarks_x = []
    est_landmarks_y = []
    landmark_keys_in_frame = []
    # Check all potential landmark keys up to max_landmark_index
    for j in range(max_landmark_index + 1):
        lm_key = L(j)
        if step_results.exists(lm_key):
            lm_point = step_results.atPoint2(lm_key)
            est_landmarks_x.append(lm_point[0])
            est_landmarks_y.append(lm_point[1])
            landmark_keys_in_frame.append(lm_key)

    lm_trace = create_est_landmarks_trace(est_landmarks_x, est_landmarks_y)
    if lm_trace:
        frame_traces.append(lm_trace)

    # 3. Generate Covariance Ellipses (if marginals available)
    if step_marginals is not None:
        # Pose ellipse
        current_pose_key = X(k)
        if step_results.exists(current_pose_key):
            try:
                pose_cov = step_marginals.marginalCovariance(current_pose_key)
                pose_mean = step_results.atPose2(current_pose_key).translation()
                frame_shapes.append(
                    create_pose_ellipse_shape(pose_mean, pose_cov, k, ellipse_scale)
                )
            except Exception as e:
                if verbose:
                    print(f"Warn: Pose {k} cov err @ step {k}: {e}")

        # Landmark ellipses
        for lm_key in landmark_keys_in_frame:
            try:
                lm_cov = step_marginals.marginalCovariance(lm_key)
                lm_mean = step_results.atPoint2(lm_key)
                lm_index = gtsam.Symbol(lm_key).index()
                frame_shapes.append(
                    create_landmark_ellipse_shape(
                        lm_mean, lm_cov, lm_index, ellipse_scale
                    )
                )
            except Exception as e:
                lm_index = gtsam.Symbol(lm_key).index()
                if verbose:
                    print(f"Warn: LM {lm_index} cov err @ step {k}: {e}")

    return frame_traces, frame_shapes


# --- Figure Configuration ---


def configure_figure_layout(
    fig: go.Figure,
    num_steps: int,
    world_size: float,
    initial_shapes: List[Dict[str, Any]],
) -> None:
    """Configures Plotly figure layout, axes, slider, buttons."""
    steps = list(range(num_steps + 1))
    sliders = [
        dict(
            active=0,
            currentvalue={"prefix": "Step: "},
            pad={"t": 50},
            steps=[
                dict(
                    label=str(k),
                    method="animate",
                    args=[
                        [str(k)],
                        dict(
                            mode="immediate",
                            frame=dict(duration=100, redraw=True),
                            transition=dict(duration=0),
                        ),
                    ],
                )
                for k in steps
            ],
        )
    ]
    updatemenus = [
        dict(
            type="buttons",
            showactive=False,
            direction="left",
            pad={"r": 10, "t": 87},
            x=0.1,
            xanchor="right",
            y=0,
            yanchor="top",
            buttons=[
                dict(
                    label="Play",
                    method="animate",
                    args=[
                        None,
                        dict(
                            mode="immediate",
                            frame=dict(duration=100, redraw=True),
                            transition=dict(duration=0),
                            fromcurrent=True,
                        ),
                    ],
                ),
                dict(
                    label="Pause",
                    method="animate",
                    args=[
                        [None],
                        dict(
                            mode="immediate",
                            frame=dict(duration=0, redraw=False),
                            transition=dict(duration=0),
                        ),
                    ],
                ),
            ],
        )
    ]

    fig.update_layout(
        title="Iterative Factor Graph SLAM Animation",
        xaxis=dict(range=[-world_size / 2 - 2, world_size / 2 + 2], constrain="domain"),
        yaxis=dict(
            range=[-world_size / 2 - 2, world_size / 2 + 2],
            scaleanchor="x",
            scaleratio=1,
        ),
        width=800,
        height=800,
        hovermode="closest",
        updatemenus=updatemenus,
        sliders=sliders,
        shapes=initial_shapes,
        legend=dict(
            traceorder="reversed",
            title_text="Legend",
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1,
        ),
    )


# --- Main Animation Orchestrator ---


def create_slam_animation(
    results_history: List[gtsam.Values],
    marginals_history: List[Optional[gtsam.Marginals]],
    num_steps: int,
    X: Callable[[int], int],
    L: Callable[[int], int],
    max_landmark_index: int,  # Required to iterate potential landmarks
    landmarks_gt_array: Optional[np.ndarray] = None,
    poses_gt: Optional[List[gtsam.Pose2]] = None,
    world_size: float = 20.0,
    ellipse_scale: float = 2.0,
    verbose_cov_errors: bool = False,
) -> go.Figure:
    """Creates a modular Plotly SLAM animation."""
    print("Generating Plotly animation...")
    fig = go.Figure()

    # 1. Add static ground truth traces to the base figure (visible always)
    gt_lm_trace = create_gt_landmarks_trace(landmarks_gt_array)
    if gt_lm_trace:
        fig.add_trace(gt_lm_trace)
    gt_path_trace = create_gt_path_trace(poses_gt)
    if gt_path_trace:
        fig.add_trace(gt_path_trace)

    # 2. Generate frames with dynamic content
    frames = []
    steps_iterable = range(num_steps + 1)
    try:
        steps_iterable = tqdm(steps_iterable, desc="Creating Frames")
    except NameError:
        pass  # tqdm optional

    for k in steps_iterable:
        step_results = results_history[k]
        step_marginals = marginals_history[k] if marginals_history else None

        frame_traces, frame_shapes = generate_frame_content(
            k,
            step_results,
            step_marginals,
            X,
            L,
            max_landmark_index,
            ellipse_scale,
            verbose_cov_errors,
        )
        frames.append(
            go.Frame(
                data=frame_traces, name=str(k), layout=go.Layout(shapes=frame_shapes)
            )
        )

    # 3. Set initial dynamic data (from frame 0) onto the base figure
    initial_dynamic_traces = []
    initial_shapes = []
    if frames:
        # Important: Add *copies* or ensure traces are regenerated if needed,
        # though Plotly usually handles this ok with frame data.
        initial_dynamic_traces = frames[0].data
        initial_shapes = frames[0].layout.shapes if frames[0].layout else []
        for trace in initial_dynamic_traces:
            fig.add_trace(trace)  # Add Est Path[0], Est Landmarks[0] traces

    # 4. Assign frames to the figure
    fig.update(frames=frames)

    # 5. Configure layout, axes, controls
    # Pass initial_shapes for the layout's starting state
    configure_figure_layout(fig, num_steps, world_size, initial_shapes)

    print("Plotly animation generated.")
    return fig
