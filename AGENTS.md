# AGENTS.md — passive-flight-core

## Mission
This repository is a thesis-grade engineering project for computing trajectories of unguided passive aircraft / aerial bombs in longitudinal nominal and perturbed motion.

The final system must support:
1. **Known objects/bombs** selected by stable ID, with authoritative tabulated aerodynamics where available.
2. **User-defined simplified layouts**, where aerodynamics are computed from geometry, then used to compute forces, moment, nominal motion, and perturbed motion.
3. **SimInTech integration** through the existing C-compatible/DLL interface.
4. **Validation**, especially comparison of bomb relative range (“относ”) and fall time against authoritative ballistic data such as BT-64.

Physics traceability, units, source provenance, reproducibility, tests, and validation are more important than feature count.

## Preserve the current architecture
Before changing code, inspect the repository, current branch, `git status`, existing tests, C API, and SimInTech code. Do not rewrite working architecture merely because another design is possible.

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
with no extra factor 1/2.

Do not silently change sign conventions, axis definitions, reference area/length, or nondimensionalization.

Trajectory motion is currently **flat Earth with constant g**. Earth radius used by the atmosphere for geometric-to-geopotential height does not make trajectory kinematics spherical.

## Atmosphere
Inspect the current implementation before changing it. The current model includes sea-level conditions, tropospheric lapse rate, geopotential altitude, an isothermal upper layer in the implemented range, ideal-gas density, speed of sound, and Sutherland viscosity.

Do not replace it casually. If a validation source uses another “normal atmosphere”, document and quantify the difference.

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

For every implemented formula, record chapter/section/equation/figure/table/page when possible. Be especially careful: some aerodynamic derivatives in chapter III use degrees and factors involving 57.3, while the C++ core uses radians.

### 2. Postnikov & Chuyko, 1979
A. G. Postnikov, V. S. Chuyko, “Методы решения прикладных задач внешней баллистики”.

Primary source for known bomb data and ballistic validation where tables are available:
- FAB-1500T data from table 1.1;
- AB-63 / BT-64 methodology;
- validation of “относ” and fall time.

Never invent missing BT-64 values. Clearly distinguish direct source values, interpolation, digitization, model-derived results, and extrapolation.

### 3. VOENMEH material
https://library.voenmeh.ru/cnau/mRAedZ3lyHq2LvD.pdf

Secondary source for justified engineering simplifications or implementation guidance. It must not silently override the primary sources.

If local source files exist under `docs/references/`, inspect them directly. If a required source is missing, say exactly what is missing instead of guessing.

## Source discipline
For every nontrivial aerodynamic relation added to code:
- identify the source;
- record equation/table/figure/page if possible;
- define variables and SI units;
- state domain of validity;
- state interpolation/digitization method;
- state approximations;
- add focused tests.

Never attribute a guessed formula to a book.

## Geometry-derived aerodynamics goal
Implement a source-traceable geometry-based aerodynamic model that can provide the coefficients required by the longitudinal equations, including as needed:
- `Cx0(M)` or a physically consistent drag decomposition;
- angle-dependent drag;
- `Cy` / normal-force slope;
- static `mz`;
- pitch damping;
- interference/downwash/retardation.

Do not replace authoritative tabulated aerodynamics of known bombs with geometry-derived approximations unless the explicit task is to compare the two methods.

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

The user-provided layout drawing is a conceptual guide to dimensional inputs. Do not guess unreadable symbols from an image.

## Digitization rules
Some Lebedev/Chernobrovkin data exists only as plots. Treat graph digitization as engineering data acquisition.

For each digitized graph:
1. record source book/page/figure;
2. define axes, units, scales, and curve parameter;
3. save raw digitized points separately from interpolation code;
4. use explicit documented interpolation;
5. never silently extrapolate;
6. verify reconstructed curves against visible source points;
7. document expected digitization error;
8. keep a reproducible digitization note/script when practical.

Prefer source tables over plots when both exist.

## Known control object: FAB-1500T
The project already contains a tabulated FAB-1500T control object.

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

Preserve this path and its tests unless evidence shows a defect.

## BT-64 validation is a major acceptance criterion
A key thesis result is agreement of computed bomb relative range (“относ”) with authoritative BT-64 data, especially for FAB-1500T when a valid reference case is available.

For each validation case record:
- source table/page;
- bomb / ballistic characteristic;
- release altitude;
- release speed;
- release angle;
- atmospheric convention if stated;
- BT-64 “относ”, m;
- BT-64 fall time, s;
- model downrange, m;
- model fall time, s;
- absolute and relative errors.

Use:
- `DeltaD = Dmodel - DBT64`
- `deltaD_percent = 100 * (Dmodel - DBT64) / DBT64`

Use multiple validation points whenever the source permits it. One matching point is insufficient evidence.

If the available source does not contain the needed low-characteristic-time BT-64 row, do not extrapolate and call it BT-64. Find the authoritative table or clearly separate a reproduced BT-64 calculation method from direct-table validation.

## Perturbed longitudinal motion
Preserve the perturbed-motion capability. The expected flow is:
1. nominal trajectory;
2. initial perturbations;
3. perturbation propagation through the dynamics / numerical linearization already implemented;
4. impact/state deviations.

Do not fake perturbations by adding arbitrary offsets after nominal integration.

Any aerodynamic change must be checked for its effect on perturbed dynamics.

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

## UI/application direction
The numerical core must remain usable without a GUI.

Eventually the app should support:
- selecting a known bomb;
- defining a supported geometry;
- entering release conditions;
- running nominal and perturbed simulations;
- viewing trajectory;
- viewing aerodynamic coefficients and provenance;
- comparing validation reference vs model.

Do not couple physics to UI widgets. Before adding a GUI framework, propose and justify the architecture/dependency.

## Numerical quality requirements
- SI units internally.
- Radians internally.
- Explicit unit conversions at boundaries.
- No hidden clamps.
- No hidden extrapolation.
- Validate finite/physical inputs.
- Deterministic output for fixed input.
- Check time-step convergence for important results.
- Preserve consistent ground-impact handling.

A finite result is not automatically a correct result. Check:
- signs;
- dimensions;
- expected monotonic trends;
- static-stability sign;
- damping sign;
- interpolation continuity;
- valid Mach/alpha/rate ranges;
- time-step sensitivity.

## Testing standard
Every meaningful implementation change needs tests.

Before claiming completion:
1. build;
2. run focused changed/new tests;
3. run full CTest;
4. run C API/DLL smoke tests where relevant;
5. run SimInTech-facing tests/build where ABI behavior changed;
6. report exact commands and results.

For tabulated/digitized data test exact nodes, interpolation, boundaries, out-of-range policy, signs, and at least one representative integrated trajectory result.

## Codex working rules
For substantial tasks:
- read this file first;
- inspect `git status`, branch, and diff;
- preserve unrelated local changes;
- use `$unlazy` if installed;
- define verifiable acceptance gates;
- implement in small coherent milestones;
- run tests per milestone;
- inspect final diff;
- explicitly report unresolved scientific/source uncertainty.

Do not stop at a plan if the user asked for implementation.

Do not push, merge, force-push, delete branches, or remotely modify `main` unless explicitly asked.

## Thesis quality bar
Every major model should have:
- equations;
- variable definitions;
- SI units;
- sign/coordinate conventions;
- source provenance;
- applicability range;
- approximation statement;
- code mapping;
- tests;
- external validation when available.

Prefer a smaller validated model to a broad undocumented one.

## First actions in every fresh Codex session
1. Read `AGENTS.md`.
2. Inspect repository state and recent relevant code.
3. Run the existing test suite to establish baseline.
4. Locate local source books/data.
5. Build a source/requirements map for the requested milestone.
6. Create a concrete plan and acceptance gates.
7. Implement unless a missing scientific definition would materially change the solution.
8. Run focused and full verification.
9. Report files changed, equations/sources used, tests, numerical results, and remaining uncertainty.

Do not assume prior ChatGPT/Codex conversation context exists. Repository files are the durable project memory.
