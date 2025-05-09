
# 📦 UKF2D — Unscented Kalman Filter for 2D Tracking

`UKF2D` is a simple implementation of a 2D Unscented Kalman Filter from scratch in Python. It is designed to estimate the **position, velocity, and acceleration** of an object (e.g., frame position) using given noisy position measurements in x and y.

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

No external libraries are required other than `numpy`.

```bash
pip install numpy
```

---

## 🧠 UKF Model Summary

- **State vector**:  
  \
  \[
  \mathbf{x} = \begin{bmatrix} x \\ y \\ v_x \\ v_y \\ a_x \\ a_y \end{bmatrix}
  \]

- **Measurements**:  
  Noisy observations of position \([x, y]\)

- **Motion model**:  
  Constant acceleration update equations.

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
    x0 = signal_x[0]
    y0 = signal_y[0]
    vx0 = 1.0
    vy0 = 1.0
    ax0 = 0.0
    ay0 = 0.0

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
- `example.py` – (Optional) Example usage and plotting

---

## 🧪 License

MIT License – feel free to use, modify, and distribute.
