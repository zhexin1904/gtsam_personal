# gtsam_plotly.py
import numpy as np
import plotly.graph_objects as go
from tqdm.notebook import tqdm  # Progress bar

import gtsam


def ellipse_path(cx, cy, sizex, sizey, angle, N=60):
    """SVG path string for an ellipse centered at (cx, cy), rotated by `angle` in degrees."""
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


# Helper to convert GTSAM covariance to Plotly ellipse parameters
def gtsam_cov_to_plotly_ellipse(cov_matrix, scale=2.0):
    """Calculates ellipse parameters (angle, width, height) from a 2x2 covariance matrix."""
    # Ensure positive definite - add small epsilon if needed
    cov = cov_matrix[:2, :2] + np.eye(2) * 1e-9
    try:
        eigvals, eigvecs = np.linalg.eigh(cov)
        # Ensure eigenvalues are positive for sqrt
        eigvals = np.maximum(eigvals, 1e-9)
    except np.linalg.LinAlgError:
        # print("Warning: Covariance matrix SVD failed, using default ellipse.") # Can be verbose
        return 0, 0.1 * scale, 0.1 * scale  # Default small ellipse

    # Width/Height are 2*scale*sqrt(eigenvalue)
    width = 2 * scale * np.sqrt(eigvals[1])
    height = 2 * scale * np.sqrt(eigvals[0])

    # Angle of the major axis (corresponding to largest eigenvalue)
    angle_rad = np.arctan2(eigvecs[1, 1], eigvecs[0, 1])
    angle_deg = np.degrees(angle_rad)

    return angle_deg, width, height


def create_slam_animation(
    results_history,  # List of gtsam.Values
    marginals_history,  # List of gtsam.Marginals or None
    landmarks_gt_array,  # Nx2 numpy array
    poses_gt,  # List of gtsam.Pose2
    num_steps,
    world_size,
    X,  # Symbol func
    L,  # Symbol func
):
    """Creates a Plotly animation of the SLAM results."""
    print("Generating Plotly animation...")

    fig = go.Figure()

    # Add Ground Truth Landmarks (static)
    fig.add_trace(
        go.Scatter(
            x=landmarks_gt_array[0, :],
            y=landmarks_gt_array[1, :],
            mode="markers",
            marker=dict(color="black", size=8, symbol="star"),
            name="Landmarks GT",
        )
    )

    # Add Ground Truth Path (static)
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

    # --- Animation Frames ---
    frames = []
    steps = list(range(num_steps + 1))

    for k in tqdm(steps):
        frame_data = []
        step_results = results_history[k]
        step_marginals = marginals_history[k]

        # Estimated Path up to step k
        est_path_x = []
        est_path_y = []
        for i in range(k + 1):
            pose_key = X(i)
            if step_results.exists(
                pose_key
            ):  # Check if pose exists in results for this step
                pose = step_results.atPose2(pose_key)
                est_path_x.append(pose.x())
                est_path_y.append(pose.y())

        frame_data.append(
            go.Scatter(
                x=est_path_x,
                y=est_path_y,
                mode="lines+markers",
                line=dict(color="red", width=2),
                marker=dict(size=4, color="red"),
                name="Path Est",
            )
        )

        # Estimated Landmarks known at step k
        est_landmarks_x = []
        est_landmarks_y = []
        landmark_keys_in_frame = []
        # Iterate through all possible landmark keys seen so far, check if present in current step's results
        all_keys = step_results.keys()
        for lm_key_val in all_keys:  # Use keys() for integer keys
            if gtsam.Symbol(lm_key_val).chr() == ord("l"):  # Check if symbol chr is 'l'
                lm_key = lm_key_val  # Use the integer key directly
                if step_results.exists(lm_key):
                    lm_pose = step_results.atPoint2(lm_key)
                    est_landmarks_x.append(lm_pose[0])
                    est_landmarks_y.append(lm_pose[1])
                    landmark_keys_in_frame.append(lm_key)

        if est_landmarks_x:
            frame_data.append(
                go.Scatter(
                    x=est_landmarks_x,
                    y=est_landmarks_y,
                    mode="markers",
                    marker=dict(color="blue", size=6, symbol="x"),
                    name="Landmarks Est",
                )
            )

        # Covariance Ellipses
        shapes = []  # List to hold ellipse shapes for this frame
        if step_marginals is not None:
            # Current Pose Covariance Ellipse
            try:
                current_pose_key = X(k)
                if step_results.exists(current_pose_key):
                    pose_cov = step_marginals.marginalCovariance(current_pose_key)
                    pose_mean = step_results.atPose2(current_pose_key).translation()
                    angle, width, height = gtsam_cov_to_plotly_ellipse(pose_cov)
                    cx, cy = pose_mean[0], pose_mean[1]
                    shapes.append(
                        dict(
                            type="path",
                            path=ellipse_path(
                                cx=cx,
                                cy=cy,
                                sizex=width,
                                sizey=height,
                                angle=angle,
                                N=60,
                            ),
                            xref="x",
                            yref="y",
                            fillcolor="rgba(255,0,255,0.2)",
                            line_color="rgba(255,0,255,0.5)",
                            name=f"Pose {k} Cov",
                        )
                    )
            except Exception as e:
                print(
                    f"Warning: Failed getting pose {k} cov ellipse at step {k}: {e}"
                )  # Can be verbose
                pass

            # Landmark Covariance Ellipses
            for lm_key in landmark_keys_in_frame:
                try:
                    lm_cov = step_marginals.marginalCovariance(lm_key)
                    lm_mean = step_results.atPoint2(lm_key)
                    angle, width, height = gtsam_cov_to_plotly_ellipse(lm_cov)
                    cx, cy = lm_mean[0], lm_mean[1]
                    index = gtsam.Symbol(lm_key).index()
                    shapes.append(
                        dict(
                            type="path",
                            path=ellipse_path(
                                cx=cx,
                                cy=cy,
                                sizex=width,
                                sizey=height,
                                angle=angle,
                                N=60,
                            ),
                            xref="x",
                            yref="y",
                            fillcolor="rgba(0,0,255,0.1)",
                            line_color="rgba(0,0,255,0.3)",
                            name=f"LM {index} Cov",
                        )
                    )
                except Exception as e:
                    index = gtsam.Symbol(lm_key).index()
                    print(
                        f"Warning: Failed getting landmark {index} cov ellipse at step {k}: {e}"
                    )  # Can be verbose

        frames.append(
            go.Frame(data=frame_data, name=str(k), layout=go.Layout(shapes=shapes))
        )

    # --- Set Initial State and Layout ---
    fig.update(frames=frames)

    # Set initial data to the first frame's data
    if frames:
        fig.add_traces(frames[0].data)
        initial_shapes = frames[0].layout.shapes if frames[0].layout else []
    else:
        initial_shapes = []

    # Define slider
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

    # Update layout
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
        updatemenus=[
            dict(
                type="buttons",
                showactive=False,
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
                    )
                ],
            )
        ],
        sliders=sliders,
        shapes=initial_shapes,  # Set initial shapes
    )
    print("Plotly animation generated.")
    return fig
