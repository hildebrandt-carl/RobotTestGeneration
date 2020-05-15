# Final Models

These are the models used as learned scoring models. Each of them is a polynomial regression model from scipy saved as a numpy object. You can load the model using:

```
poly_features = np.load("..._poly_features.npy", allow_pickle=True).item()
poly_reg = np.load("..._ridgecv_model.npy", allow_pickle=True).item()
```

These models then take in the vector velocities in and out of a waypoint and give back the expected maximum deviation of the quadrotor on which they were trained on. 

## Quadrotor naming convention

The Controllers are named as follows:

* **speed-2_minsnap0**: Unstable waypoint controller
* **speed-1_minsnap0**: Stable waypoint controller
* **speed2_minsnap0**: Fixed velocity controller 2m/s
* **speed5_minsnap0**: Fixed velocity controller 5m/s
* **speed10_minsnap0**: Fixed velocity controller 10m/s
* **speed-1_minsnap-1**: Minsnap controller