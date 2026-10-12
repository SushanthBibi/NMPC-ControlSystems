# Simulation results

These are the results from the default parameters in `model/thermal_params.m`.
They were simulated with GNU Octave, so the NMPC used `fminsearch`. With the
Optimization Toolbox, MATLAB uses `fmincon` instead, so its numbers can differ
slightly.

Each run lasts 1800 s. "Stability" is the standard deviation of the coolant
temperature `T_c` after warm-up. The set-point is 90 °C and the limit is 105 °C.

| Scenario | Controller | T_c std [K] | T_c RMSE [K] | T_c peak [°C] | T_o peak [°C] | Fan [kWh] | Pump [kWh] | Fuel [kg] | Warm-up [s] |
|---|---|---|---|---|---|---|---|---|---|
| highway (25 °C) | conventional | 1.24 | 6.31 | 89.3 | 93.7 | 0.000 | 0.203 | 13.821 | – |
| | PID | 0.69 | 0.69 | 91.7 | 100.5 | 0.000 | 0.003 | 13.755 | – |
| | **NMPC** | **0.40** | **0.40** | 90.6 | 99.9 | 0.000 | 0.004 | **13.755** | – |
| hill climb (38 °C) | conventional | 6.40 | 6.81 | 97.0 | 113.9 | 2.985 | 0.280 | 21.910 | – |
| | PID | 3.15 | 3.20 | 94.3 | 108.2 | 4.128 | 0.267 | 22.130 | – |
| | **NMPC** | 3.01 | 3.26 | 96.2 | 110.9 | **1.876** | 0.270 | **21.677** | – |
| urban (30 °C) | conventional | 3.13 | 6.89 | 89.5 | 92.9 | 0.000 | 0.103 | 9.707 | – |
| | PID | 1.65 | 2.00 | 92.2 | 95.8 | 0.000 | 0.003 | 9.659 | – |
| | **NMPC** | **0.75** | **1.13** | 90.7 | 94.4 | 0.001 | 0.016 | 9.661 | – |
| cold start (5 °C) | conventional | 0.24 | 6.70 | 84.0 | 89.1 | 0.000 | 0.171 | 14.836 | 364 |
| | PID | 0.98 | 0.99 | 90.6 | 98.8 | 0.000 | 0.003 | 14.776 | 364 |
| | **NMPC** | 0.96 | 0.97 | 90.2 | 98.8 | 0.000 | 0.018 | **14.780** | 363 |

## Observations

- **Hill climb** is the hardest case, and the NMPC gains the most there. It uses
  **37 % less fan energy than the conventional fan clutch** and 55 % less than
  PID, and burns **1.1 % less fuel** than conventional. Its temperature is about
  twice as stable as conventional (σ 3.0 K vs 6.4 K). It achieves this by running
  the fan continuously at part speed, because fan power rises with the cube of
  speed. The on/off clutch instead runs the fan at full speed in bursts. The PID
  holds the temperature tighter to 90 °C and pays for it with fan energy.
- **Highway and urban:** no fan is needed. The NMPC and PID both save pump energy
  by running an electric pump slowly and opening the valve, where the belt pump
  follows engine speed. The NMPC gives the tightest temperature control.
- The **conventional wax thermostat** settles around 84 °C, a proportional-control
  offset, so the oil runs colder and friction is higher. This is why the PID and
  NMPC use slightly less fuel even when they do not use the fan.
- **Warm-up time** is about the same for all three, because during warm-up the
  valve stays closed whichever controller is used. Warm-up speed is set by the
  engine heat, not by the controller.

## Effect of preview (hill climb, horizon 24 × 5 s = 2 min, earlier cost weights)

| NMPC variant | Fan [kWh] | Fuel [kg] | T_c std [K] |
|---|---|---|---|
| without preview (current load held constant) | 2.600 | 21.824 | 2.46 |
| with preview of speed and load | **2.267** | **21.757** | 2.54 |

Knowing the future load cut fan energy by about 13 %. With the default
60 s horizon the preview effect is small, because a 10-minute climb is mostly
longer than the look-ahead. Longer horizons or route-based (GPS) preview are
worth exploring.

## Lessons learned while tuning

With a purely quadratic tracking cost and the engine far below the set-point
(cold start), the optimiser found it "worth" running the fan to load the
engine and warm it with extra fuel. The fuel use was +2.3 %. The tracking term
now uses a Huber penalty: quadratic within ±5 K of the set-point and linear
outside it. Warm-up is now driven by the fuel/friction term and the fan stays
off.

## Model validation (synthetic data; see `sim/validate_model.m`)

| | RMSE T_c [K] | R² T_c | RMSE T_o [K] | R² T_o |
|---|---|---|---|---|
| nominal model | 9.87 | 0.913 | 11.06 | 0.923 |
| calibrated model | 0.63 | 1.000 | 0.60 | 1.000 |

The calibration recovered the injected parameter errors:
coolant heat ×1.128 (true value 1.12), radiator UA ×0.879 (0.85) and oil heat
×1.097 (1.10). This demonstrates the workflow only. Real validation needs
measured data.
