
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import cv2
import numpy as np
import imageio

from ukf2D import UKF2D


#== unscentedKalmanFilter ==============================
def unscentedKalmanFilter(signal_x, signal_y, timestamp):
    """
    Apply 2D unscented Kalman Filter on given 2d noisy signal x,y 

    Args:
        signal_x (vector): Loaded panorama image.
        signal_y (vector): X coordinate of the crop center.
        timestamp (float): Y coordinate of the crop center.
        
    Returns:
        estimated state vectors - x, y, vx, vy, ax, ay 
    """

    # Init state vector: 
    x0 = signal_x[0]
    y0 = signal_y[0]
    vx0 = 1.0
    vy0 = 1.0
    ax0 = 0.0
    ay0 = 0.0

    # Init 2D Unscented Kalman Filter object: 
    ukf = UKF2D([x0, y0, vx0, vy0, ax0, ay0], timestamp)
    
    #Init state estimations over time: 
    x_est = []
    y_est = []
    vx_est = []
    vy_est = []
    accx_est = []
    accy_est = []

    for x_measure, y_measure in zip(signal_x, signal_y):
        # Estimate next frame state vector:
        x, y, vx, vy, ax, ay = ukf.step([x_measure, y_measure])
        x_est.append(x)
        y_est.append(y)
        vx_est.append(vx)
        vy_est.append(vy)
        accx_est.append(ax)
        accy_est.append(ay)

    return x_est, y_est, vx_est, vy_est, accx_est, accy_est

#== crop_from_panorama =================================
def crop_from_panorama(pano_img, center_x, center_y, crop_width, crop_height, ball_x, ball_y):
    """
    Crops a region from the given panoramic image centered at (center_x, center_y).

    Args:
        pano_img (PIL.Image.Image): Loaded panorama image.
        center_x (float): X coordinate of the crop center.
        center_y (float): Y coordinate of the crop center.
        crop_width (int): Width of the crop (in pixels).
        crop_height (int): Height of the crop (in pixels).

    Returns:
        PIL.Image.Image: Cropped image.
    """
    img_width, img_height = pano_img.size

    # Calculate bounding box
    left = int(center_x - crop_width / 2)
    top = int(center_y - crop_height / 2)
    right = left + crop_width
    bottom = top + crop_height

    # Clamp to image boundaries
    left = max(0, left)
    top = max(0, top)
    right = min(img_width, right)
    bottom = min(img_height, bottom)

    # Compute ball position relative to crop
    ball_x_crop = int(ball_x - left)
    ball_y_crop = int(ball_y - top)

    # Crop and return
    return pano_img.crop((left, top, right, bottom)), ball_x_crop, ball_y_crop

#== evaluate_crop_performance ==========================
def evaluate_crop_performance(frame_center_x, frame_center_y, crop_width_vec, ball_x, ball_y, panorama_width, panorama_height):
    """
    Evaluates how well the cropped regions contain the ball.

    Args:
        frame_center_x (list): Crop center x-coordinates per frame.
        frame_center_y (list): Crop center y-coordinates per frame.
        crop_width_vec (list): Width of the crop.
        ball_x (list): Ball x-coordinates per frame in panorama coordinates.
        ball_y (list): Ball y-coordinates per frame in panorama coordinates.

    Returns:
        dict: Dictionary with performance metrics.
    """
    total_frames = len(frame_center_x)
    ball_inside_frame_count = 0
    ball_inside_panorama_count = 0
    distances = []

    for cx, cy, bx, by, crop_width in zip(frame_center_x, frame_center_y, ball_x, ball_y, crop_width_vec):

        # Crop each frame according to the filterred location:
        crop_height = crop_width * 9 / 16
        
        left = cx - crop_width / 2
        right = cx + crop_width / 2
        top = cy - crop_height / 2
        bottom = cy + crop_height / 2

        # Check if the ball is inside the full panorama image:
        inside_panorama = 0 <= bx < panorama_width and 0 <= by < panorama_height
        if inside_panorama:
            ball_inside_panorama_count += 1

        # Check if the ball is also inside the crop frame:
        inside_frame = left <= bx <= right and top <= by <= bottom
        if inside_panorama and inside_frame:
            ball_inside_frame_count += 1

        # Distance from ball to crop center
        distance = ((bx - cx)**2 + (by - cy)**2)**0.5
        distances.append(distance)

    coverage_ratio = ball_inside_frame_count / ball_inside_panorama_count
    avg_distance = sum(distances) / total_frames
    max_distance = max(distances)
    std_distance = np.std(distances)

    return {
        "total_frames": total_frames,
        "ball_inside_ratio": round(coverage_ratio, 3),
        "avg_ball_distance_to_center": round(avg_distance, 2),
        "max_ball_distance_to_center": round(max_distance, 2),
        "std_ball_distance": round(std_distance, 2)
    }

#== visualize_data =================================
def visualize_data(signal_x, signal_y, state_estimations):

    frame_center_x_filter, frame_center_y_filter, vx_est, vy_est, accx_est, accy_est = state_estimations

    # Visuallize data over time:
    plt.figure(figsize=(10, 4))

    plt.subplot(211)
    plt.plot(signal_x, label='Input X')
    plt.plot(frame_center_x_filter, label='Filtered X')
    plt.title("Frame Center Over Time")
    plt.xlabel("Frame Index")
    plt.ylabel("X Position (pixels)")
    plt.legend()
    plt.grid(True)

    plt.subplot(212)
    plt.plot(signal_y, label='Input Y')
    plt.plot(frame_center_y_filter, label='Filtered Y')
    plt.xlabel("Frame Index")
    plt.ylabel("Y Position (pixels)")
    plt.legend()
    plt.grid(True)


    plt.figure(figsize=(10, 4))

    plt.subplot(411)
    plt.plot(vx_est, label='Estimated Velocity in x')
    plt.xlabel("Frame Index")
    plt.ylabel("Velocity X (pixels/frame)")
    plt.legend()
    plt.grid(True)

    plt.subplot(412)
    plt.plot(vy_est, label='Estimated Velocity in y')
    plt.xlabel("Frame Index")
    plt.ylabel("Velocity Y (pixels/frame)")
    plt.legend()
    plt.grid(True)

    plt.subplot(413)
    plt.plot(accx_est, label='Estimated acceleration in x')
    plt.xlabel("Frame Index")
    plt.ylabel("Acceleration x (pixels/frame^2)")
    plt.legend()
    plt.grid(True)

    plt.subplot(414)
    plt.plot(accy_est, label='Estimated acceleration in y')
    plt.xlabel("Frame Index")
    plt.ylabel("Acceleration y (pixels/frame^2)")
    plt.legend()
    plt.grid(True)

    plt.show()

#== show_result =================================
def show_result(pano_img, frame_center_x_filter, frame_center_y_filter, ball_x, ball_y, crop_width_vec, fps = 25, save_to_gif=True):
    """
    Run cropped images with ball position to visualize the result.

    Args:
        pano_img (PIL image):           Panorama image
        frame_center_x_filter (list): x positions over time
        frame_center_y_filter (list): y positions over time
        ball_x (list):                  ball x positions over time
        ball_y (list):                  ball y positions over time
        crop_width_vec (list):          width of each crop over time
        fps (int):                      frames per second.
        save_to_gif (bool):             if True - create a gif file and save to disk

    """
    
    # Create a list to store the frames
    gif_frames = []

    delay = int(1000 / fps)  # Delay between frames in milliseconds
    
    target_width = np.min(crop_width_vec)
    target_height = target_width * 9 / 16
    target_size = (int(target_width), int(target_height))

    for center_x, center_y, ball_x, ball_y, crop_width in zip(frame_center_x_filter, frame_center_y_filter, ball_x, ball_y, crop_width_vec):

        # Crop each frame according to the filterred location:
        crop_height = crop_width * 9 / 16
        
        crop_frame, ball_x_crop, ball_y_crop = crop_from_panorama(pano_img, center_x, center_y, crop_width, crop_height, ball_x, ball_y)

        # Convert to OpenCV image (BGR)
        frame_bgr = cv2.cvtColor(np.array(crop_frame), cv2.COLOR_RGB2BGR)
    
        # Draw ball as a white circle if it's within the cropped area
        if 0 <= ball_x_crop < crop_width and 0 <= ball_y_crop < crop_height:
            cv2.circle(frame_bgr, (ball_x_crop, ball_y_crop), radius=6, color=(255, 255, 255), thickness=-1)

        #Adjust frame size:
        frame_bgr = cv2.resize(frame_bgr, target_size)

        if save_to_gif:
            # Convert BGR (OpenCV) to RGB (imageio expects RGB)
            rgb_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            gif_frames.append(rgb_frame)

        # Show the frame
        cv2.imshow("Cropped Frame", frame_bgr)

        # Wait and break if 'q' is pressed
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

    if save_to_gif:
        imageio.mimsave('../Output/FrameBoundaries.gif', gif_frames, duration=0.04)  # duration in seconds per frame

#============== MAIN =============================================

# Read data files:
frame_boundaries = pd.read_csv("../Data/FrameBoundaries.csv")
ball_gt = pd.read_csv("../Data/Ball_GT.csv")

# Load image
pano_img = Image.open("../Data/Pano.jpg")
panorama_width, panorama_height = pano_img.size

# Clean spaces from headers:
frame_boundaries.columns = frame_boundaries.columns.str.strip()
ball_gt.columns = ball_gt.columns.str.strip()

# Calculate timestamp between frames:
dt = frame_boundaries['timestamp'][1] - frame_boundaries['timestamp'][0]

# Apply Kalman filter combined with average filter to stabilize the camera:
state_estimations = unscentedKalmanFilter(frame_boundaries['x'], frame_boundaries['y'], dt)
frame_center_x_filter, frame_center_y_filter, vx_est, vy_est, accx_est, accy_est = state_estimations

print(evaluate_crop_performance(frame_center_x_filter, frame_center_y_filter, frame_boundaries['width'], ball_gt['x'], ball_gt['y'], panorama_width, panorama_height))

# Visuallize data over time:
visualize_data(frame_boundaries['x'], frame_boundaries['y'], state_estimations)

# Visualize the stabilized frame live with the ball on it:
show_result(pano_img, frame_center_x_filter, frame_center_y_filter, ball_gt['x'], ball_gt['y'], frame_boundaries['width'], fps = 25)

# Update the dataframe and save the result:
frame_boundaries['x'] = frame_center_x_filter
frame_boundaries['y'] = frame_center_y_filter
frame_boundaries.to_csv("../Output/FrameBoundariesResult.csv", index=False)
