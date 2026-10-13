# PID Control of a Truck Engine Cooling System

Report companion for `run_pid_report.m`.

---

## 1. The control problem

A heavy-duty truck engine must keep its **coolant temperature near 90 °C**:

- **Too hot:** above about 105 °C the engine overheats, the coolant boils and parts get damaged.
- **Too cold:** the oil gets thick, friction rises and the engine burns more fuel.

The controller has three actuators that remove heat:

| Actuator | Symbol | Range | Power cost |
|---|---|---|---|
| Thermostat valve (sends coolant through the radiator) | x_v | 0–1 | almost zero |
| Coolant pump (how fast coolant flows) | u_p | 0.15–1 | up to 1.5 kW |
| Radiator fan (blows air through the radiator) | u_f | 0–1 | up to 25 kW |

The controller **cannot** change the **disturbances**: engine load, engine
speed, vehicle speed and outside temperature.

### Block diagram (closed loop / negative feedback)

```
             e(t)        ┌──────────┐  y   ┌────────────┐  u   ┌──────────────┐
 T_ref ──►(Σ)──────────►│   PID    │─────►│ Split range │─────►│   Engine +   │──┬──► T_c
 (90°C)   ▲ +  −         └──────────┘      └────────────┘      │ cooling sys. │  │
          │                                                     └──────▲───────┘  │
          │                                                            │ load,    │
          │                                                            │ ambient  │
          └────────────────── temperature sensor ◄─────────────────────┴──────────┘
```

## 2. Plant model (what the PID is controlling)

The model is a lumped-parameter energy balance:
*rate of change of stored heat = heat in − heat out*.

**Engine coolant** (the controlled variable):

$$C_c \frac{dT_c}{dt} = Q_{engine} + Q_{oil} - \dot m\, x_v\, c_p (T_c - T_r) - UA_{amb}(T_c - T_{amb})$$

**Radiator:**

$$C_r \frac{dT_r}{dt} = \dot m\, x_v\, c_p (T_c - T_r) - Q_{rad}$$

**Oil:**

$$C_o \frac{dT_o}{dt} = Q_{friction} - Q_{oil\,cooler} - UA_{o}(T_o - T_{amb})$$

where

- $Q_{engine} \approx 0.22 \times$ fuel power. About 22 % of the fuel's energy goes into the coolant.
- $\dot m = \dot m_{max} \cdot u_p$ is the coolant flow, set by the pump.
- $Q_{rad} = \varepsilon\, \dot m_{air} c_{p,air} (T_r - T_{amb})$, where the air flow $\dot m_{air}$ comes from the vehicle's speed plus the fan.
- Fan power $= 25\text{ kW} \cdot u_f^3$ (affinity law, so half speed costs 1/8 of the power).

For the PID, this model is simply **"the plant"**: a slow, first-order-like
thermal process with a time constant of a few minutes.

## 3. The PID controller

### 3.1 Error

$$e(t) = T_c(t) - T_{ref}$$

A positive error means the engine is too hot and needs more cooling.
(This sign convention is used because the actuators *remove* heat.)

### 3.2 Continuous PID law

$$y(t) = \underbrace{K_p\, e(t)}_{P} + \underbrace{K_i \int_0^t e(\tau)\, d\tau}_{I} + \underbrace{K_d \frac{de(t)}{dt}}_{D}$$

| Term | Reacts to | Effect |
|---|---|---|
| **P** proportional | the present error | Fast reaction, but leaves a **steady-state error (offset)** |
| **I** integral | the accumulated past error | **Removes the offset**; too much causes overshoot and oscillation |
| **D** derivative | the error's rate of change (predicts the future) | **Damps** the response and reduces the peak; sensitive to noise |

### 3.3 Discrete implementation (as in `controllers/ctrl_pid.m`)

The controller runs every $T_s = 1$ s (sample $k$):

$$P_k = K_p\, e_k$$

$$I_k = I_{k-1} + K_i\, e_k\, T_s \qquad \text{(rectangular integration)}$$

$$D_k = \alpha D_{k-1} + (1-\alpha)\, K_d \frac{T_{c,k} - T_{c,k-1}}{T_s}, \qquad \alpha = \frac{T_f}{T_f + T_s}$$

$$y_k = \text{sat}_{[0,\,3]}\left(P_k + I_k + D_k\right)$$

Notes:
- **Derivative on measurement:** D uses $T_c$ instead of $e$, so a sudden set-point change does not cause a spike ("derivative kick").
- **Derivative filter:** a first-order low-pass with $T_f = 10$ s stops sensor noise from being amplified.

### 3.4 Anti-windup (conditional integration)

The actuators saturate: the output cannot go below 0 % or above 100 %. When
they are saturated and the error keeps pushing in the same direction, the
integrator is frozen:

$$\text{if } (y_{unsat} \ge 3 \text{ and } e > 0) \text{ or } (y_{unsat} \le 0 \text{ and } e < 0): \quad I_k = I_{k-1}$$

Without this, during a cold start (the error is about −85 K for minutes) the
integrator "winds up" to a huge negative value. Once the engine is warm, it
takes minutes to unwind, and the engine overheats in the meantime
(Experiment 4).

### 3.5 Split-range output

A single PID output $y \in [0, 3]$ drives the three actuators, **cheapest first**:

| y | Valve x_v | Pump u_p | Fan u_f |
|---|---|---|---|
| 0 → 1 | 0 → 100 % | 15 % | 0 |
| 1 → 2 | 100 % | 15 → 100 % | 0 |
| 2 → 3 | 100 % | 100 % | 0 → 100 % |

### 3.6 Tuning values (`model/thermal_params.m`)

| Parameter | Value | Unit |
|---|---|---|
| $K_p$ | 0.30 | 1/K |
| $K_i$ | 0.006 | 1/(K·s) (integral time $T_i = K_p/K_i = 50$ s) |
| $K_d$ | 2.0 | s/K (derivative time $T_d = K_d/K_p \approx 6.7$ s) |
| $T_f$ | 10 | s |
| $T_s$ | 1 | s |

These were tuned by trial-and-error simulation: increase $K_p$ until the response
is fast without oscillating, add $K_i$ to remove the offset, then add a small $K_d$
to reduce the peak.

## 4. Performance measures

| Measure | Definition |
|---|---|
| Peak deviation | largest $|T_c - T_{ref}|$ after a disturbance |
| Settling time | time until $T_c$ stays within ±1 K of $T_{ref}$ |
| Steady-state error | average error over the final 2 minutes |
| RMSE | $\sqrt{\frac{1}{N}\sum (T_c - T_{ref})^2}$, overall regulation quality |
| Std. deviation | how much $T_c$ fluctuates |
| Fan + pump energy | $\int (P_{fan} + P_{pump})\,dt$ in kWh |

## 5. Experiments and results

Run `run_pid_report.m`. The numbers below were produced in GNU Octave; MATLAB
gives the same results.

### Experiment 1: Load step (P vs PI vs PID) → `figures/1_load_step_P_PI_PID.png`
The truck drives at 80 km/h. Engine load jumps from 30 % to 80 % at t = 600 s.

| Controller | Peak deviation [K] | Settling time (±1 K) [s] | Steady-state error [K] |
|---|---|---|---|
| P only | 4.06 | never settles | **4.06** |
| PI | 1.96 | 65 | 0.00 |
| PID | **1.58** | 74 | 0.00 |

**Conclusions:**
- A P-only controller always leaves an offset. It needs a non-zero error to produce a non-zero output.
- The I term removes the offset.
- The D term cuts the peak by about 20 %. Its benefit is small because thermal systems are slow and smooth.

### Experiment 2: P, I, D contributions → `figures/2_PID_terms.png`
- When the load steps up, P and D react instantly.
- I builds up slowly and, at steady state, provides **all** of the output while P and D return to zero.

### Experiment 3: Set-point tracking → `figures/3_setpoint_step.png`
The set-point changes 90 → 95 → 90 °C. The figure shows that $T_c$ follows,
and how the actuators open and close in order (valve, then pump, then fan).

### Experiment 4: Integrator windup → `figures/4_anti_windup.png`
Cold start from 5 °C.

| | Peak T_c |
|---|---|
| Without anti-windup | **157.5 °C** (severe overshoot) |
| With anti-windup | 90.6 °C |

The model does not include coolant boiling, so 157 °C is not physically
realistic. The point is the trend: **without anti-windup the engine
overheats badly.**

### Experiment 5: PID vs conventional cooling → `figures/5_PID_vs_conventional.png`, `figures/6_rmse_bar.png`
The conventional system is a wax thermostat (in effect P-only), an on/off fan
clutch and a belt-driven pump.

| Scenario | Controller | T_c std [K] | RMSE [K] | Peak [°C] | Fan+pump [kWh] | Fuel [kg] |
|---|---|---|---|---|---|---|
| Highway | Conventional | 1.24 | 6.31 | 89.3 | 0.203 | 13.821 |
| | **PID** | 0.69 | **0.69** | 91.7 | **0.003** | **13.755** |
| Hill climb | Conventional | 6.40 | 6.81 | 97.0 | **3.265** | **21.910** |
| | **PID** | **3.15** | **3.20** | 94.3 | 4.395 | 22.130 |
| Urban | Conventional | 3.13 | 6.89 | 89.5 | 0.103 | 9.707 |
| | **PID** | **1.65** | **2.00** | 92.2 | **0.003** | **9.659** |
| Cold start | Conventional | 0.24 | 6.70 | 84.0 | 0.171 | 14.836 |
| | **PID** | 0.98 | **0.99** | 90.6 | **0.003** | **14.776** |

**Conclusions:**
- **PID regulates temperature 2 to 9 times more accurately** (RMSE). The conventional wax thermostat is a P-only controller, so it settles at about 84 °C (a 6 K offset).
- On the highway, urban and cold-start runs the PID also uses less pump energy and slightly less fuel. The warmer oil means less friction.
- **The limitation:** on the hill climb the PID uses **more** fan energy than the on/off fan. It reacts only to the current error and holds 90 °C tightly whatever the cost. This motivates the project's next step: a **predictive controller (MPC)** that can see the hill coming and trade temperature accuracy against energy.

## 6. Simulink model

```matlab
cd simulink
build_thermal_simulink('pid', 'hill')
```

This builds `thermal_pid.slx` using Simulink's **Discrete PID Controller**
block (same gains, output limited to 0–3, clamping anti-windup), a Sum block
for the error, the plant as a MATLAB function, and an Integrator. Take a
screenshot of the block diagram for the report.
