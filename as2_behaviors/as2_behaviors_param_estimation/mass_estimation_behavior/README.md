
# mass_estimation_behavior

The `mass_estimation_behavior` is part of the **as2_behaviors_param_estimation** package in Aerostack2.  
Its purpose is to estimate the **mass of the aerial platform** in real-time during flight, using sensor data from the IMU and the `actuator_command/thrust` topic.

---

## How Mass Estimation Works

The behavior subscribes to the linear acceleration along the **z-axis** and stores the data.  
In each iteration of the behavior, the mean of the acceleration is computed, and the mass is estimated using the following equation:

$$
\text{mass} = \frac{\text{thrust}}{a_z\_mean}
$$

If there is a significant difference compared to the last computed value, the new estimation is stored in a stack.  
The estimation is considered valid only if it lies within the **minimum** and **maximum** values set in the configuration file.  

The final `estimated_mass` is computed by applying a **low-pass filter** to the mean of the last `n_samples`.

Other parameters can be modified in the configuration file, see the `config/config_default.yaml` for detailed explanations.

---

## Launching the Behavior

There are two main ways to run the `mass_estimation_behavior`:

### 1. Autostart (automatic action)

Run:

```bash
ros2 launch as2_behaviors_param_estimation mass_estimation_behavior_auto_launch.py
```
### 2. Manual 
Run:
```bash
ros2 launch as2_behaviors_param_estimation mass_estimation_behavior_launch.py
```
This launch file only starts the behavior node. You must manually send the action goal using:

```bash
ros2 action send_goal /<namespace>/MassEstimationBehavior as2_msgs/action/MassEstimation '{"active_behavior": true}'
```

## Debud info
    - `mass_estimation_topic`: Publishes all estimated mass values.
    - `mass_filtered_topic`: Publishes the mass after filtering.
    - `mass_update_topic`: Publishes the updated mass data sent to the controller.