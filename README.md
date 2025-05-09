
# 📦 Stream Boundary Analysis and Noise Reduction
The following algorithm objective is to generate smooth stream boundaries for video frames by processing frame boundary data. 
The algorithm uses Unscented Kalman Filter to reduce noise, ensure a coherent transition across frames, with minimum time delay.
The algorithm result and analysis can be found below.

---

# 📦 UKF2D — Unscented Kalman Filter for 2D Tracking

`UKF2D` is a simple implementation of a 2D Unscented Kalman Filter from scratch in Python. It is designed to estimate the **position, velocity, and acceleration** of an object using given noisy position measurements in x and y.

This filter is useful when the object's **velocity is not constant** — it assumes a **constant-acceleration motion model**, making it more robust to sudden changes in velocity.

More information can be found in this paper:
https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf

---

## 🚀 Features

- ✅ 6D state: `[x, y, vx, vy, ax, ay]`
- ✅ Models non-constant velocity (supports acceleration)
- ✅ From-scratch UKF: no dependencies beyond `numpy`
- ✅ Modular design using a class (`UKF2D`)
- ✅ Tunable process and measurement noise
- ✅ Easily extensible for other applications

---

## 🛠️ Installation

The follwing python libraries are needed:

```bash
pip install numpy
pip install pandas
pip install matplotlib
pip install PIL
pip install opencv-python
pip install imageio
```
- Download the repository and save it on your PC. 
- Open the repository and run the python script: frameBoundaries.py
- You should see the plot of the input signals and estimated signals.
- When you close the plot windows, You able to see the simulation of the cropped frames with the ball location.
- Output: The output files will be created in the Output directory (also attached for my running) - A gif file of the cropped frame simulation and the output csv. 

---

## 📋 Usage Example

```python
from ukf2d import UKF2D

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

    # Init 2D Unscented Kalman Filter: 
    vx0 = 1.0
    vy0 = 1.0
    ax0 = 0.0
    ay0 = 0.0

    ukf = UKF2D([signal_x[0], signal_y[0], vx0, vy0, ax0, ay0], timestamp)
    
    for x_measure, y_measure in zip(signal_x, signal_y):
        # Estimate next frame state vector:
        x, y, vx, vy, ax, ay = ukf.step([x_measure, y_measure])
        print("Current state:",  x, y, vx, vy, ax, ay)
```

---

## ⚙️ Parameters

| Parameter         | Description                                           |
|------------------|-------------------------------------------------------|
| `x0`              | initial state values          |
| `process_noise`   | Process noise scale (affects responsiveness), can be float or list of float per state |
| `measurement_noise` | Noise scale for the measurement model, can be float or list of float per measure    |

---

## 📈 Output

Each call to `ukf.step(z)` returns the current state estimate:
```python
[x, y, vx, vy, ax, ay]
```
You can extract just the estimated position or all states as needed.

---


## 📚 Files

- `ukf2d.py` – The UKF2D class implementation
- `framesBoundaries.py` – Use ukf2d to estimate frame boundaries and create result plotting and analysis

---

## 📈 Algorithm Result and Analysis

Running the algorithm with the following filter parameter:

process_noise=0.01, 
measurement_noise=[1000.0, 100.0]

gives the optimal output in terms of noise vs. time delay. The chosen measurement noise for x and y are different since the signal noise standard deviation in x is much heigher then in y.

The plot below illustrates the behavior of the input frame center signal before and after applying the Kalman filter:

![localization result](Output/localiztion_result.png)

The raw signal is highly noisy, exhibiting significant fluctuations that obscure the underlying trend. The noise may be due to various factors, such as measurement errors or external disturbances, making it difficult to extract meaningful information from the data.

Upon applying the Kalman filter, the signal is significantly smoother. The filter effectively reduces the noise, revealing a much clearer representation of the signal’s true behavior. This demonstrates the Kalman filter's ability to perform noise reduction while maintaining the core characteristics of the signal.

One notable side effect of the Kalman filter is the slight delay in the filtered signal. This delay occurs due to the filter’s reliance on previous and current measurements to estimate the state, which can introduce a phase lag, especially in real-time applications. While this delay is minimal, it is an important consideration when applying the filter in time-sensitive systems.

The plot below illustrate the velocity and acceleration, also estimated by this filter

![state estimation](Output/state_estimation.png)

Additional interesting observation is the Ball-to-Frame Ratio Analysis - by analyzing this ratio, we can assess the ball's movement within the frame, evaluate how well the ball is tracked, and detect any instances where the ball may be out of the frame.
The analysis focuses on the ratio of the ball’s position relative to the frame, where the signal represents the measured center location of the frame, and the ball’s position is provided for each frame.
We define Ball-to-Frame ratio as the number of croped frames where the ball was inside the crop area vs. the total number of frames where the ball was anywhere inside the full panorama image.

Using the above parameter to the filter, we got the following results:
- 60% of the cropped frames included the ball
- The average distance from the ball to the frame center is 85 pixels

Conclusion
The ball-to-frame ratio provides valuable insight into how frequent the ball is positioned within the frame over time, aiding in tracking and ensuring consistent coverage throughout the sequence of frames.

---
