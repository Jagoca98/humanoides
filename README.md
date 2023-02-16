# Humanoid Robots :robot:

## Webots :lady_beetle:

This project contains the files needed to move Teo's robot arm. It contains two parts, the basic `teo_main_cpp` part that consists of moving the arm up and down in succession, and an extra part `teo_extra_cpp` that performs circular movements with the arm. This project is available as a git repository.

```bash
git clone https://github.com/Jagoca98/humanoides.git
```

### `teo_main_cpp`

The file contains the functions for calculating the direct and inverse kinematics of the two-degree-of-freedom arm. To check that it works correctly, draw a trajectory consisting of three points on the `Z-axis` in a loop.

### `teo_extra_cpp`

In this case a 2D circular trajectory has been implemented in the X-Z plane. Polar coordinates with a discretisation of $\theta$ between 0 and $2\pi$ have been used.

$$\begin{cases} x = C_x + R\cdot \cos(\theta) & \\ y = C_y + R\cdot \sin(\theta) &  \end{cases} \text{, with }  \theta \in \left[0,2\pi\right]$$

where $\left(C_x, C_y\right)$ is the center of the drawn circumference.







