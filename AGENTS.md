# AGENTS.md — passive-flight-core

## Mission

This repository is a thesis-grade engineering project for computing trajectories of unguided passive aircraft / aerial bombs in longitudinal nominal and perturbed motion.

The final system must support:

1. **Known objects/bombs** selected by stable ID, with authoritative tabulated aerodynamics where available.
2. **User-defined simplified layouts**, where aerodynamics are computed from geometry, then used to compute forces, moment, nominal motion, and perturbed motion.
3. **SimInTech integration** through the existing C-compatible/DLL interface.
4. **Validation**, especially comparison of bomb relative range (“относ”) and fall time against authoritative ballistic data such as BT-64.

Physics traceability, units, source provenance, reproducibility, tests, and validation are more important than feature count.

---

## Preserve the current architecture

Before changing code, inspect:

- the repository;
- current branch;
- `git status`;
- existing implementation;
- existing tests;
- C API;
- SimInTech code;
- relevant documentation and source material.

Do not rewrite working architecture merely because another design is possible.

Important existing concepts include:

- `ObjectModel`
- `ObjectPassport`
- `ObjectRegistry`
- `AerodynamicModel`
- `PreliminaryAerodynamicModel`
- `TabulatedAerodynamicModel`
- `ObjectAerodynamicsAdapter`
- `StandardAtmosphere`
- `NominalLongitudinalDynamics`
- `PerturbedLongitudinalDynamics`
- `ForwardEulerSimulator`
- trajectory / impact analysis
- C API
- SimInTech adapters/blocks

Known-object tabulated aerodynamics and geometry-derived aerodynamics must coexist behind stable interfaces.

Before modifying an existing file, read its current version first.

Never overwrite or discard unrelated local changes.

---

## Current longitudinal model

Conceptual nominal state:

`[V, Theta, omega_z, theta_pitch, x, H]^T`

with:

`alpha = theta_pitch - Theta`.

Baseline equations:

- `dV/dt = -X/m - g*sin(Theta)`
- `dTheta/dt = Y/(m*V) - g*cos(Theta)/V`
- `domega_z/dt = Mz/Iz`
- `dtheta_pitch/dt = omega_z`
- `dx/dt = V*cos(Theta)`
- `dH/dt = V*sin(Theta)`

Loads:

- `q = rho*V^2/2`
- `X = q*S*Cx`
- `Y = q*S*Cy`
- `Mz = q*S*Lref*mz`

Core angles are radians unless a source adapter explicitly converts degrees.

For the current tabulated FAB-1500T source convention:

`omega_bar_z = omega_z * Lref / V`

with no extra factor `1/2`.

Do not silently change:

- sign conventions;
- axis definitions;
- reference area;
- reference length;
- aerodynamic coefficient definitions;
- angular units;
- nondimensionalization.

Trajectory motion is currently **flat Earth with constant g**.

Earth radius used by the atmosphere for geometric-to-geopotential height does not make trajectory kinematics spherical.

---

## Atmosphere

Inspect the current implementation before changing it.

The current model includes:

- sea-level conditions;
- tropospheric lapse rate;
- geometric-to-geopotential altitude conversion;
- an isothermal upper layer in the implemented range;
- ideal-gas density;
- speed of sound;
- Sutherland viscosity.

Do not replace or modify the atmosphere casually.

If a validation source uses another definition of “normal atmosphere”, document the difference and quantify its effect when practical.

The trajectory model and atmosphere model must not be conceptually confused:

- trajectory kinematics currently use flat Earth;
- Earth radius is used only where explicitly required by the atmosphere implementation.

---

## Engineering sources and priority

Use sources in this order unless the task explicitly requires otherwise.

### 1. Lebedev & Chernobrovkin, 1973

A. A. Lebedev, L. S. Chernobrovkin, “Динамика полета беспилотных летательных аппаратов”.

Primary theoretical source for longitudinal dynamics and geometry-based aerodynamics:

- body normal force;
- isolated lifting surfaces;
- interference;
- downwash;
- retardation;
- static pitching moment;
- pitch damping.

For every implemented formula, record chapter/section/equation/figure/table/page when possible.

Be especially careful: some aerodynamic derivatives in chapter III use degrees and factors involving `57.3`, while the C++ core uses radians.

Never transfer a derivative expressed per degree directly into code that expects a derivative per radian.

### 2. Postnikov & Chuyko, 1979

A. G. Postnikov, V. S. Chuyko, “Методы решения прикладных задач внешней баллистики”.

Primary source for known bomb data and ballistic validation where tables are available:

- FAB-1500T data from table 1.1;
- AB-63 / BT-64 methodology;
- ballistic characteristics;
- characteristic time;
- validation of “относ”;
- validation of fall time.

Never invent missing BT-64 values.

Clearly distinguish:

- direct source values;
- interpolation;
- digitization;
- reconstructed values;
- model-derived results;
- extrapolation.

An extrapolated or model-derived value must never be presented as a direct BT-64 table value.

### 3. VOENMEH material

Secondary source:

`docs/references/Voenmeh_aerodynamics.pdf`

Use it for justified engineering simplifications or implementation guidance.

It must not silently override the primary sources.

If local source files exist under `docs/references/`, inspect them directly.

If a required source is missing, unreadable, incomplete, or does not contain the required data, say exactly what is missing instead of guessing.

---

## Source discipline

For every nontrivial aerodynamic or ballistic relation added to code:

1. identify the source;
2. record equation/table/figure/page when possible;
3. define variables;
4. define SI units;
5. state sign conventions where relevant;
6. state domain of validity;
7. state interpolation/digitization method;
8. state approximations;
9. state extrapolation policy;
10. add focused tests.

Never attribute a guessed or reconstructed formula to a book.

If a formula has been algebraically transformed from the source, document enough information to trace the implementation back to the original expression.

If two sources disagree, do not silently choose one. Record the discrepancy and determine which source convention applies to the current model.

---

## Geometry-derived aerodynamics goal

Implement a source-traceable geometry-based aerodynamic model that can provide the coefficients required by the longitudinal equations, including as needed:

- `Cx0(M)` or a physically consistent drag decomposition;
- angle-dependent drag;
- `Cy` / normal-force slope;
- static `mz`;
- pitch damping;
- interference;
- downwash;
- retardation.

Do not replace authoritative tabulated aerodynamics of known bombs with geometry-derived approximations unless the explicit task is to compare the two methods.

The geometry-derived path and known-object tabulated path must remain conceptually separate even if they share interfaces.

---

## Initial configurable geometry

Keep the first geometry scope intentionally small:

- body length and diameter;
- nose type: `Conical` or `Ogival`;
- nose length/fineness needed by the source method;
- rear-body/tail geometry if required;
- main lifting-surface geometry;
- stabilizer geometry;
- component axial positions / aerodynamic-center information required by the equations;
- reference area and reference length;
- mass;
- center of mass;
- pitch moment of inertia.

Do not build general CAD-like freedom until the limited model is validated.

The user-provided layout drawing is a conceptual guide to dimensional inputs.

Do not guess unreadable symbols or dimensions from an image.

---

## Digitization rules

Some Lebedev/Chernobrovkin and ballistic data may exist only as plots or scanned tables.

Treat graph/table digitization as engineering data acquisition.

For each digitized graph or table:

1. record source book/page/figure/table;
2. define axes/columns;
3. define units;
4. define scales and curve parameters;
5. save raw digitized points separately from interpolation code when practical;
6. use explicit documented interpolation;
7. never silently extrapolate;
8. verify reconstructed data against visible source points;
9. document expected digitization error;
10. keep a reproducible digitization note/script when practical.

Prefer source tables over plots when both exist.

Never manually alter digitized values merely to improve agreement with the numerical model.

---

## Known control object: FAB-1500T

The project contains a tabulated FAB-1500T control object.

Current source passport:

- ID: `FAB_1500T_POSTNIKOV_1979`
- mass: 1519 kg
- length: 3.46 m
- diameter: 0.58 m
- center of mass from nose: 1.16 m
- longitudinal source inertia: 76.8 kg*m^2
- equatorial/pitch inertia used for longitudinal motion: 1122.3 kg*m^2
- reference area: bomb midsection
- pitching-moment reference length: bomb length

Source tables contain:

- `Cx0(M)`
- `Cx^(alpha^2)(M)`
- `Cy^alpha(M)`
- `mz^alpha(M)`
- `mz^(omega_bar)(M)`

Model form:

- `Cx = Cx0 + Cx_alpha2 * alpha^2`
- `Cy = Cy_alpha * alpha`
- `mz = mz_alpha * alpha + mz_omega_bar * omega_bar_z`

Preserve this path and its tests unless source evidence shows a defect.

Do not tune FAB-1500T aerodynamic coefficients merely to improve agreement with BT-64.

If disagreement exists, investigate:

- source convention;
- atmosphere;
- ballistic characteristic;
- initial conditions;
- coefficient normalization;
- interpolation;
- reference area;
- reference length;
- equations of motion;
- numerical integration;
- definition of “относ”.

---

## BT-64 validation is a major acceptance criterion

A key thesis result is agreement of computed bomb relative range (“относ”) with authoritative BT-64 data, especially for FAB-1500T when a valid reference case is available.

The primary validation quantity is bomb relative range / downrange:

`Dmodel = x(t_impact)`

compared against the authoritative BT-64 “относ” for the same initial conditions and ballistic characteristic.

For each validation case record:

- source table/page;
- bomb / ballistic characteristic;
- characteristic time if applicable;
- ballistic coefficient if applicable;
- reference resistance law;
- release altitude;
- release speed;
- release angle;
- atmospheric convention if stated;
- BT-64 “относ”, m;
- BT-64 fall time, s;
- model downrange, m;
- model fall time, s;
- absolute range error;
- relative range error;
- absolute fall-time error;
- relative fall-time error.

Use:

`DeltaD = Dmodel - DBT64`

and:

`deltaD_percent = 100 * (Dmodel - DBT64) / DBT64`

For fall time use analogously:

`DeltaT = Tmodel - TBT64`

and:

`deltaT_percent = 100 * (Tmodel - TBT64) / TBT64`

Use multiple validation points whenever the source permits it.

One matching point is insufficient evidence.

The goal is to validate the dependency:

`D = D(H0, V0, releaseAngle, ballisticCharacteristic)`

over a useful set of source-supported conditions.

If the available source does not contain the required low-characteristic-time BT-64 row, do not extrapolate and call it BT-64.

Either:

- find the authoritative table; or
- implement a separately documented reproduction of the BT-64 calculation method.

A reproduced BT-64 calculation and a direct BT-64 table lookup are different validation categories and must be reported separately.

Never tune the numerical model against individual BT-64 validation points.

---

## Perturbed longitudinal motion

Preserve the perturbed-motion capability.

The expected flow is:

1. nominal trajectory;
2. initial perturbations;
3. perturbation propagation through the dynamics / numerical linearization already implemented;
4. impact/state deviations.

Do not fake perturbations by adding arbitrary offsets after nominal integration.

Any aerodynamic change must be checked for its effect on perturbed dynamics.

When a change affects the nominal right-hand side, assume that it may also affect numerical linearization and perturbed motion until tests show otherwise.

---

## SimInTech integration must remain working

SimInTech integration is a required deliverable.

The existing C/DLL path must continue to allow blocks to receive release conditions and an object/bomb selector through the existing ABI and return summary/trajectory results.

Requirements:

- preserve backward compatibility unless a versioned change is intentional;
- keep C API separate from C++ domain logic;
- keep SimInTech adapters thin;
- do not hard-code one bomb;
- object selection must use the registry;
- update smoke tests if public ABI behavior changes.

Do not leak C++ implementation types through the C API.

Changes to core physics must not require unnecessary changes to SimInTech-facing interfaces.

---

## UI/application direction

The numerical core must remain usable without a GUI.

Eventually the app should support:

- selecting a known bomb;
- defining a supported geometry;
- entering release conditions;
- running nominal simulations;
- running perturbed simulations;
- viewing trajectory;
- viewing aerodynamic coefficients;
- viewing source provenance;
- comparing validation reference vs model.

Do not couple physics to UI widgets.

Before adding a GUI framework, propose and justify the architecture/dependency.

---

## Numerical quality requirements

Use:

- SI units internally;
- radians internally;
- explicit unit conversions at boundaries;
- deterministic output for fixed input.

Do not use:

- hidden clamps;
- hidden extrapolation;
- undocumented empirical tuning;
- silent unit conversion;
- silent sign changes.

Validate finite and physically meaningful inputs.

Check time-step convergence for important results.

Preserve consistent ground-impact handling.

A finite result is not automatically a correct result.

Check:

- signs;
- dimensions;
- expected monotonic trends;
- static-stability sign;
- damping sign;
- interpolation continuity;
- valid Mach ranges;
- valid alpha ranges;
- valid angular-rate ranges;
- time-step sensitivity;
- physical plausibility of integrated trajectories.

When validating an engineering quantity, compare it against an independent reference whenever possible.

---

## Russian code documentation

All newly created classes, structs, public functions, methods, physical fields, and non-obvious physical variables must have short and technically precise comments in Russian.

Comments must explain physical or architectural meaning rather than merely repeat C++ syntax.

### Classes and structures

Every new class or structure must have a short Russian comment describing its purpose and role in the mathematical or software model.

Example:

```cpp
/// Модель стандартной атмосферы для вычисления параметров воздуха
/// по геометрической высоте.
class StandardAtmosphere {
    // ...
};