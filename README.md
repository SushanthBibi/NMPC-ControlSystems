# NMPC Predictive Thermal Management for Heavy-Duty Trucks

MATLAB / Simulink implementation of the project
*"Sustainable, Compact and Lightweight Thermal Management System for Heavy-Duty
Trucks to Improve Fuel Economy and Reduce CO2 Emissions"*: a **Nonlinear Model
Predictive Controller (NMPC)** for the engine cooling system, compared against
today's **conventional** cooling (wax thermostat, belt pump, fan clutch) and a
**basic PID** controller.

The code only needs base MATLAB and Simulink. If the Optimization Toolbox is
installed, the NMPC uses `fmincon` and runs much faster. Without it, the NMPC
uses `fminsearch` from base MATLAB. Every `.m` file also runs in GNU Octave.

---

## 1. Quick start

```matlab
run_comparison                      % all scenarios x all controllers, KPI table + plots
addpath model controllers sim
validate_model                      % model calibration / validation (RMSE, R^2)
out = simulate_system('nmpc','hill');   % a single run
cd simulink; run_simulink_demo      % builds the .slx models and simulates them
```

## 2. Repository layout

| Folder / file | Purpose | Methodology step (from the PDF) |
|---|---|---|
| `model/thermal_params.m` | All parameters (engine, radiator, fan, pump, controller tuning) | 1, 2 |
| `model/thermal_model.m` | Nonlinear thermal model `dx = f(x,u,d)` | 3 |
| `model/rk4_step.m` | Runge-Kutta 4 integrator | 3 |
| `sim/validate_model.m` | Calibrate + validate the model, report RMSE and R² | 4 |
| `controllers/nmpc_controller.m` | The NMPC | 5 |
| `sim/drive_cycle.m` | Operating scenarios: highway, hill, urban, cold start | 6 |
| `controllers/ctrl_conventional.m` | Wax thermostat + belt pump + on/off fan clutch | 7 |
| `controllers/ctrl_pid.m` | Split-range PID with anti-windup | 7 |
| `sim/simulate_system.m`, `sim/compute_kpis.m`, `sim/plot_results.m` | Closed-loop simulation, KPIs, plots | 6, 7 |
| `run_comparison.m` | Main script | 6, 7 |
| `simulink/` | Script-built Simulink models (`build_thermal_simulink.m`) | 8 (platform) |

---

## 3. Background: the control concepts used

### 3.1 Plant, inputs, outputs, disturbances
A control system has a **plant** (here, the engine plus its cooling circuit),
**manipulated variables** that the controller sets (fan, pump, valve), **disturbances**
that the controller cannot set (engine load, vehicle speed, ambient temperature),
and **outputs** to keep in check (coolant and oil temperature). The PDF's variable
table maps onto these groups as follows:

| PDF group | In this code |
|---|---|
| Inputs: coolant, oil temperature | states `x = [T_c; T_r; T_o]` (measured) |
| Inputs: engine speed, load, vehicle speed, ambient temperature | disturbance `d = [N; Tq; v; T_amb]` |
| Preview variables: predicted speed and load | `drive_cycle(...).d(t + k*Ts)`, passed to the NMPC |
| Manipulated variables: fan, pump, thermostat | `u = [u_f; u_p; x_v]`, each 0…1 |
| Outputs: stability, peak, fan/pump power, fuel, warm-up time | `compute_kpis.m` |

### 3.2 Feedback and PID
**Feedback** measures the output, compares it with the set-point
(error `e = T_c − T_ref`), and corrects the input. A **PID** controller computes

```
y = Kp·e + Ki·∫e dt + Kd·de/dt
```

- **P** (proportional) reacts to the present error.
- **I** (integral) removes the steady-state error that a plain P controller (or a wax thermostat) leaves.
- **D** (derivative) damps the response. Here it acts on the measurement and is low-pass filtered so that sensor noise is not amplified.
- **Anti-windup:** the actuators saturate at 0 % and 100 %. While they are saturated, the integrator is frozen (conditional integration). Without this the integrator keeps growing and the temperature overshoots afterwards.
- **Split range:** a single PID output `y ∈ [0,3]` drives three actuators, cheapest first: the valve over 0–1, the pump over 1–2 and the fan over 2–3.

### 3.3 Why a conventional thermostat is "P-only"
A wax thermostat opens in proportion to temperature: 0 % at 82 °C and 100 % at
95 °C. It is a mechanical **proportional controller**, so it has a permanent
**offset** and settles wherever the heat balance puts it, around 84 °C. The fan
clutch is an **on/off (bang-bang) controller with hysteresis** (on at 96 °C, off
at 90 °C), which makes the temperature oscillate. These are the baseline
behaviours the PID and NMPC are compared against.

### 3.4 Model Predictive Control
MPC repeats these four steps at every sample `Ts`:

1. Measure the current state `x`.
2. Use a model to **predict** the temperatures over a horizon of `Np` steps, for a candidate input sequence, using the **preview** of future speed and load.
3. **Optimise** that input sequence to minimise a cost while respecting the limits.
4. Apply only the **first** input, then go back to step 1 (receding horizon).

It is "nonlinear" (NMPC) because the prediction model in step 2 is the nonlinear
model in §4. No linearisation is used.

Cost minimised by `nmpc_controller.m`:

```
J = Σ_k [ w_T H(T_c,k − T_ref)            temperature tracking (Huber: quadratic within ±5 K, linear outside)
        + w_fuel · fuel_k [g/s]           fuel, including fan + pump load and cold-oil friction
        + w_soft · (limit violation)² ]   soft limits: T_c ≤ 103 °C, T_o ≤ 123 °C
  + Σ w_du (Δu)²                          smooth actuator moves
subject to  u_min ≤ u ≤ u_max             hard bounds (fan 0–1, pump 0.15–1, valve 0–1)
```

Settings: `Ts = 5 s`, `Np = 12` (a 60 s look-ahead), and **move blocking**
`[2 10]`, which leaves only 2 × 3 = 6 free decision variables so that it solves
quickly.

How the NMPC saves energy:
- Fan power rises with the **cube** of fan speed, so a steady 50 % fan uses 1/8 of the power of 100 %. On/off control wastes this; the NMPC exploits it.
- It uses the cheapest actuator first: the valve is free, the pump costs at most 1.5 kW and the fan at most 25 kW.
- **Preview:** before a known hill climb it pre-cools while cooling is still cheap, then lets the temperature rise within the safe band instead of running the fan at maximum.

---

## 4. Nonlinear thermal model (`model/thermal_model.m`)

It is a lumped-parameter model with three thermal masses:

```
                   Q_cool (from combustion)
                        │
   ┌──────────┐  mdot·x_v   ┌────────────┐  air (ram + fan)
   │  Engine  │ ──────────► │  Radiator  │ ───────► Q_rad to ambient
   │  T_c     │ ◄────────── │  T_r       │
   └──────────┘             └────────────┘
       ▲ │ bypass (1−x_v)·mdot
  Q_oc │
   ┌──────────┐
   │   Oil    │ ◄── Q_oil (friction)
   │   T_o    │
   └──────────┘
```

```
C_c dT_c/dt = Q_cool + Q_oc − ṁ x_v c_p (T_c − T_r) − UA_amb (T_c − T_amb)
C_r dT_r/dt = ṁ x_v c_p (T_c − T_r) − Q_rad
C_o dT_o/dt = Q_oil − Q_oc − UA_oamb (T_o − T_amb)
```

- Fuel power: `P_fuel = [(Tq·ω + P_fan + P_pump)/η + P_idle·N/N_idle] · (1 + k_fric·max(0, 90 − T_o))`. Cold oil means more friction and therefore more fuel, which is why fast warm-up matters.
- Heat into the coolant and the oil: `Q_cool = 0.22·P_fuel`, `Q_oil = 0.04·P_fuel + extra friction`.
- Radiator (ε-NTU method): `ṁ_air = ρ·A·k_ram·v + ṁ_fan,max·u_f`, `UA = UA₀(ṁ_air/ṁ_ref)^0.8`, `ε = 1 − exp(−UA/(ṁ_air c_pa))`, `Q_rad = ε ṁ_air c_pa (T_r − T_amb)`.
- Pump flow `ṁ = ṁ_max·u_p`. Power: `P_fan = 25 kW·u_f³`, `P_pump = 1.5 kW·u_p³` (affinity laws).
- Oil cooler: `Q_oc = UA_oc0·(0.3 + 0.7·u_p)·(T_o − T_c)`.

**The parameter values are representative of a 13 L heavy-duty diesel, not
measured.** `validate_model.m` contains the full calibration and validation
workflow: identify scale factors on the first half of a dataset, then report
RMSE and R² on the second half. By default it runs on *synthetic* reference
data, created by deliberately mis-scaling the parameters and adding sensor
noise, to show that the procedure works. **To actually validate the model**
(PDF step 4), call `validate_model('your_data.csv')` with test-bench or vehicle
logs in the column format documented in the file.

---

## 5. Scenarios (`sim/drive_cycle.m`)

| Scenario | Description | Ambient |
|---|---|---|
| `highway` | 80 km/h motorway cruise with rolling-hill load (±20 %) | 25 °C |
| `hill` | Cruise, then a 10 min full-load climb at 35 km/h, descent, cruise | 38 °C |
| `urban` | 12 stop-and-go delivery cycles | 30 °C |
| `coldstart` | Warm-up from a 5 °C soak | 5 °C |

## 6. Results

Run `run_comparison` to produce the KPI table: coolant and oil temperature
stability (standard deviation), RMSE to the set-point, peak temperatures, fan
and pump energy (kWh), fuel (kg) and warm-up time. Results measured while
developing this code are recorded in [`docs/results.md`](docs/results.md).

## 7. Simulink

`simulink/build_thermal_simulink.m` builds the closed loop from basic library
blocks: Clock, Mux/Demux, Integrator, Sum, Constant, Discrete PID Controller,
Interpreted MATLAB Function, To Workspace and Scope. It then saves the model as
`thermal_<controller>.slx`.

- **Plant:** `Interpreted MATLAB Function` (`sl_plant`) gives `dx = f(x,u,d)` → `Integrator` gives `x`.
- **PID loop:** `T_c − T_ref` → **Discrete PID Controller** block (clamping anti-windup, output 0…3) → split range → actuators.
- **NMPC loop:** `[x; d; t]` → `Interpreted MATLAB Function` (`sl_controller`, sample time 5 s) → actuators.

The models are generated by a script rather than committed as binary `.slx`
files, so they match your MATLAB release and the repository stays diff-able.

## 8. Limitations and next steps

- The parameters are literature-typical, not measured. Calibrate them with real data (step 4).
- Perfect preview is assumed. In a vehicle, the predicted speed and load would come from GPS/map data (route topography) or ADAS. Set `p.mpc.preview = false` to see the controller without preview.
- Fan and pump are modelled as ideal speed-controlled actuators, with the same shaft-power cost for electric and mechanical drives.
- For real-time use, replace `fminsearch` with `fmincon` (SQP) or a dedicated solver such as CasADi/IPOPT or acados, and generate code.

## 9. References (from the project brief)

1. MDPI *Energies* 19(13):3123: NMPC thermal management for fuel-cell heavy-duty trucks. https://www.mdpi.com/1996-1073/19/13/3123
2. *Applied Energy*: MPC for auxiliary (cooling) systems of 40 t and 60 t trucks; reported auxiliary fuel reduction of 26 % and 38 %. https://www.sciencedirect.com/science/article/abs/pii/S0306261916317494
3. MDPI *Energies* 12(2):259. https://www.mdpi.com/1996-1073/12/2/259
4. IFAC-PapersOnLine. https://www.sciencedirect.com/science/article/pii/S2405896318325357
5. IFAC Proceedings Volumes. https://www.sciencedirect.com/science/article/pii/S147466701536818X

> The development environment could not download the papers (network policy),
> so the model and controller here are built from standard lumped-parameter
> cooling-system physics and textbook PID/MPC theory, following the
> methodology in the project brief. Cross-check the parameter values and the
> cost-function structure against these papers.
