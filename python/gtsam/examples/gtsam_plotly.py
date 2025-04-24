# gtsam_plotly_modular.py
import numpy as np
import plotly.graph_objects as go
from tqdm.notebook import tqdm  # Progress bar
from typing import List, Optional, Tuple, Dict, Any

import gtsam

# --- Ellipse Calculation Helpers (Mostly unchanged) ---


def ellipse_path(
    cx: float, cy: float, sizex: float, sizey: float, angle: float, N: int = 60
) -> str:
    """
    Generates an SVG path string for an ellipse.

    Args:
        cx: Center x-coordinate.
        cy: Center y-coordinate.
        sizex: Full width of the ellipse along its major axis.
        sizey: Full height of the ellipse along its minor axis.
        angle: Rotation angle in degrees.
        N: Number of points to approximate the ellipse.

    Returns:
        SVG path string.
    """
    angle_rad = np.radians(angle)
    t = np.linspace(0, 2 * np.pi, N)
    x = (sizex / 2) * np.cos(t)
    y = (sizey / 2) * np.sin(t)

    x_rot = cx + x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rot = cy + x * np.sin(angle_rad) + y * np.cos(angle_rad)

    path = (
        f"M {x_rot[0]},{y_rot[0]} "
        + " ".join(f"L{x_},{y_}" for x_, y_ in zip(x_rot[1:], y_rot[1:]))
        + " Z"
    )
    return path


def gtsam_cov_to_plotly_ellipse(
    cov_matrix: np.ndarray, scale: float = 2.0
) -> Tuple[float, float, float]:
    """
    Calculates ellipse parameters (angle, width, height) from a 2x2 covariance matrix.

    Args:
        cov_matrix: The 2x2 covariance matrix (or larger, only top-left 2x2 used).
        scale: Scaling factor for the ellipse size (e.g., 2.0 for 2-sigma).

    Returns:
        Tuple containing (angle_degrees, width, height).
    """
    # Ensure positive definite - add small epsilon if needed
    cov = cov_matrix[:2, :2] + np.eye(2) * 1e-9
    try:
        eigvals, eigvecs = np.linalg.eigh(cov)
        # Ensure eigenvalues are positive for sqrt
        eigvals = np.maximum(eigvals, 1e-9)
    except np.linalg.LinAlgError:
        # print("Warning: Covariance matrix SVD failed, using default ellipse.") # Optional warning
        return 0, 0.1 * scale, 0.1 * scale  # Default small ellipse

    # Width/Height are 2*scale*sqrt(eigenvalue) (using full width/height)
    width = (
        2 * scale * np.sqrt(eigvals[1])
    )  # Major axis corresponds to largest eigenvalue
    height = (
        2 * scale * np.sqrt(eigvals[0])
    )  # Minor axis corresponds to smallest eigenvalue

    # Angle of the major axis (eigenvector corresponding to largest eigenvalue eigvals[1])
    angle_rad = np.arctan2(eigvecs[1, 1], eigvecs[0, 1])
    angle_deg = np.degrees(angle_rad)

    return angle_deg, width, height


# --- Plotting Element Creation Helpers ---


def _add_ground_truth_traces(
    fig: go.Figure, landmarks_gt_array: np.ndarray, poses_gt: List[gtsam.Pose2]
) -> None:
    """Adds static ground truth landmark and path traces to the figure."""
    # Ground Truth Landmarks
    if landmarks_gt_array is not None and landmarks_gt_array.size > 0:
        fig.add_trace(
            go.Scatter(
                x=landmarks_gt_array[0, :],
                y=landmarks_gt_array[1, :],
                mode="markers",
                marker=dict(color="black", size=8, symbol="star"),
                name="Landmarks GT",
            )
        )

    # Ground Truth Path
    if poses_gt:
        gt_path_x = [p.x() for p in poses_gt]
        gt_path_y = [p.y() for p in poses_gt]
        fig.add_trace(
            go.Scatter(
                x=gt_path_x,
                y=gt_path_y,
                mode="lines",
                line=dict(color="gray", width=1, dash="dash"),
                name="Path GT",
            )
        )


def _create_ellipse_shape_dict(
    cx: float,
    cy: float,
    angle: float,
    width: float,
    height: float,
    fillcolor: str,
    line_color: str,
    name: str,
) -> Dict[str, Any]:
    """Creates the dictionary required for a Plotly ellipse shape."""
    return dict(
        type="path",
        path=ellipse_path(cx=cx, cy=cy, sizex=width, sizey=height, angle=angle, N=60),
        xref="x",
        yref="y",
        fillcolor=fillcolor,
        line_color=line_color,
        name=name,  # Note: name isn't directly displayed for shapes, but good for metadata
    )


def _create_single_frame_data(
    k: int,
    step_results: gtsam.Values,
    step_marginals: Optional[gtsam.Marginals],
    X: callable,
    L: callable,
    ellipse_scale: float = 2.0,
    verbose: bool = False,
) -> Tuple[List[go.Scatter], List[Dict[str, Any]]]:
    """
    Creates the traces and shapes for a single animation frame.

    Args:
        k: The current step index.
        step_results: gtsam.Values for this step.
        step_marginals: gtsam.Marginals for this step (or None).
        X: Symbol function for poses.
        L: Symbol function for landmarks.
        ellipse_scale: Scaling factor for covariance ellipses.
        verbose: If True, print warnings for covariance errors.

    Returns:
        A tuple containing (list_of_traces, list_of_shapes).
    """
    traces = []
    shapes = []

    # 1. Estimated Path up to step k
    est_path_x = []
    est_path_y = []
    for i in range(k + 1):
        pose_key = X(i)
        if step_results.exists(pose_key):
            pose = step_results.atPose2(pose_key)
            est_path_x.append(pose.x())
            est_path_y.append(pose.y())

    traces.append(
        go.Scatter(
            x=est_path_x,
            y=est_path_y,
            mode="lines+markers",
            line=dict(color="red", width=2),
            marker=dict(size=4, color="red"),
            name="Path Est",  # Legend entry for the whole path
        )
    )

    # 2. Estimated Landmarks known at step k
    est_landmarks_x = []
    est_landmarks_y = []
    landmark_keys_in_frame = []
    all_keys = step_results.keys()
    for key_val in all_keys:
        symbol = gtsam.Symbol(key_val)
        if symbol.chr() == ord("l"):  # Check if it's a landmark symbol
            # Check existence again (though keys() implies existence)
            if step_results.exists(key_val):
                lm_point = step_results.atPoint2(key_val)
                est_landmarks_x.append(lm_point[0])
                est_landmarks_y.append(lm_point[1])
                landmark_keys_in_frame.append(key_val)

    if est_landmarks_x:
        traces.append(
            go.Scatter(
                x=est_landmarks_x,
                y=est_landmarks_y,
                mode="markers",
                marker=dict(color="blue", size=6, symbol="x"),
                name="Landmarks Est",  # Legend entry for all estimated landmarks
            )
        )

    # 3. Covariance Ellipses (if marginals available)
    if step_marginals is not None:
        # Current Pose Covariance Ellipse
        current_pose_key = X(k)
        if step_results.exists(current_pose_key):
            try:
                pose_cov = step_marginals.marginalCovariance(current_pose_key)
                pose_mean = step_results.atPose2(current_pose_key).translation()
                angle, width, height = gtsam_cov_to_plotly_ellipse(
                    pose_cov, scale=ellipse_scale
                )
                shapes.append(
                    _create_ellipse_shape_dict(
                        cx=pose_mean[0],
                        cy=pose_mean[1],
                        angle=angle,
                        width=width,
                        height=height,
                        fillcolor="rgba(255,0,255,0.2)",
                        line_color="rgba(255,0,255,0.5)",
                        name=f"Pose {k} Cov",
                    )
                )
            except Exception as e:
                if verbose:
                    print(
                        f"Warning: Failed getting pose {k} cov ellipse at step {k}: {e}"
                    )

        # Landmark Covariance Ellipses
        for lm_key in landmark_keys_in_frame:
            try:
                lm_cov = step_marginals.marginalCovariance(lm_key)
                lm_mean = step_results.atPoint2(lm_key)
                angle, width, height = gtsam_cov_to_plotly_ellipse(
                    lm_cov, scale=ellipse_scale
                )
                symbol = gtsam.Symbol(lm_key)
                shapes.append(
                    _create_ellipse_shape_dict(
                        cx=lm_mean[0],
                        cy=lm_mean[1],
                        angle=angle,
                        width=width,
                        height=height,
                        fillcolor="rgba(0,0,255,0.1)",
                        line_color="rgba(0,0,255,0.3)",
                        name=f"LM {symbol.index()} Cov",
                    )
                )
            except Exception as e:
                symbol = gtsam.Symbol(lm_key)
                if verbose:
                    print(
                        f"Warning: Failed getting landmark {symbol.index()} cov ellipse at step {k}: {e}"
                    )

    return traces, shapes


def _configure_figure_layout(
    fig: go.Figure,
    num_steps: int,
    world_size: float,
    initial_shapes: List[Dict[str, Any]],
) -> None:
    """Configures the Plotly figure's layout, axes, slider, and buttons."""

    steps = list(range(num_steps + 1))

    # Slider
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
                        [str(k)],  # Frame name
                        dict(
                            mode="immediate",
                            frame=dict(
                                duration=100, redraw=True
                            ),  # Redraw needed for shapes
                            transition=dict(duration=0),
                        ),
                    ],
                )
                for k in steps
            ],
        )
    ]

    # Buttons
    updatemenus = [
        dict(
            type="buttons",
            showactive=False,
            buttons=[
                dict(
                    label="Play",
                    method="animate",
                    args=[
                        None,  # Animate all frames
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
                        [None],  # Stop animation
                        dict(
                            mode="immediate",
                            frame=dict(duration=0, redraw=False),
                            transition=dict(duration=0),
                        ),
                    ],
                ),
            ],
            direction="left",
            pad={"r": 10, "t": 87},
            x=0.1,
            xanchor="right",
            y=0,
            yanchor="top",
        )
    ]

    # Layout settings
    fig.update_layout(
        title="Iterative Factor Graph SLAM Animation",
        xaxis=dict(
            range=[-world_size / 2 - 2, world_size / 2 + 2],
            constrain="domain",  # Keep aspect ratio when zooming
        ),
        yaxis=dict(
            range=[-world_size / 2 - 2, world_size / 2 + 2],
            scaleanchor="x",  # Ensure square aspect ratio
            scaleratio=1,
        ),
        width=800,
        height=800,
        hovermode="closest",
        updatemenus=updatemenus,
        sliders=sliders,
        shapes=initial_shapes,  # Set initial shapes from frame 0
        # Add legend if desired
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


# --- Main Animation Function (Orchestrator) ---


def create_slam_animation(
    results_history: List[gtsam.Values],
    marginals_history: List[Optional[gtsam.Marginals]],
    num_steps: int,
    X: callable,
    L: callable,
    landmarks_gt_array: Optional[np.ndarray] = None,
    poses_gt: Optional[List[gtsam.Pose2]] = None,
    world_size: float = 20.0,
    ellipse_scale: float = 2.0,
    verbose_cov_errors: bool = False,
) -> go.Figure:
    """
    Creates a Plotly animation of the SLAM results in a modular way.

    Args:
        results_history: List of gtsam.Values, one per step.
        marginals_history: List of gtsam.Marginals or None, one per step.
        num_steps: The total number of steps (results_history should have length num_steps + 1).
        X: Symbol function for poses (e.g., lambda i: gtsam.symbol('x', i)).
        L: Symbol function for landmarks (e.g., lambda j: gtsam.symbol('l', j)).
        landmarks_gt_array: Optional Nx2 numpy array of ground truth landmark positions.
        poses_gt: Optional list of gtsam.Pose2 ground truth poses.
        world_size: Approximate size of the world for axis scaling.
        ellipse_scale: Scaling factor for covariance ellipses (e.g., 2.0 for 2-sigma).
        verbose_cov_errors: If True, print warnings for covariance calculation errors.

    Returns:
        A plotly.graph_objects.Figure containing the animation.
    """
    print("Generating Plotly animation...")

    fig = go.Figure()

    # 1. Add static ground truth elements
    _add_ground_truth_traces(fig, landmarks_gt_array, poses_gt)

    # 2. Create frames for animation
    frames = []
    steps_iterable = range(num_steps + 1)

    # Use tqdm for progress bar if available
    try:
        steps_iterable = tqdm(steps_iterable, desc="Creating Frames")
    except NameError:
        pass  # tqdm not installed or not in notebook env

    for k in steps_iterable:
        step_results = results_history[k]
        step_marginals = marginals_history[k] if marginals_history else None

        # Create traces and shapes for this specific frame
        frame_traces, frame_shapes = _create_single_frame_data(
            k, step_results, step_marginals, X, L, ellipse_scale, verbose_cov_errors
        )

        # Create the Plotly frame object
        frames.append(
            go.Frame(
                data=frame_traces,
                name=str(k),  # Name used by slider/buttons
                layout=go.Layout(
                    shapes=frame_shapes
                ),  # Shapes are part of layout per frame
            )
        )

    # 3. Set initial figure state (using data from frame 0)
    if frames:
        # Add traces from the first frame as the initial state
        for trace in frames[0].data:
            fig.add_trace(trace)
        initial_shapes = frames[0].layout.shapes if frames[0].layout else []
    else:
        initial_shapes = []

    # 4. Assign frames to the figure
    fig.update(frames=frames)

    # 5. Configure overall layout, slider, buttons
    _configure_figure_layout(fig, num_steps, world_size, initial_shapes)

    print("Plotly animation generated.")
    return fig
