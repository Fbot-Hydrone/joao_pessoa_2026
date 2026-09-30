---
tags: [hydrone, phase4, plan]
---
# Degraded Sensing

Back to [[Hydrone]]. The robust layer above basic LIO odometry: detecting when
a sensor is degraded or actively lying, and modelling sensor error online
rather than trusting every reading at face value. See [[Phase 4 Pipeline]] for
where this sits, and [[Sensor Fallbacks]] for what the drone does once a
sensor is declared unhealthy.

Ivo's framing: the basic layer assumes every sensor is telling the truth. The
robust layer assumes some of them are lying, and has to figure out which,
without another sensor to arbitrate — the only outside reference is the
drone's own past observations.

## Sensor health states

A simple state machine per sensor (lidar, Pixhawk IMU/attitude, ESC RPM, and
later the RGB camera):

- **Healthy** — readings agree with cross-checks and with the physics prior
  within the normal error margin.
- **Suspect** — readings are inconsistent with other sources or with the
  physics prior, but not yet enough evidence to declare a failure. Weight
  down, keep watching.
- **Lying** — readings are confidently wrong (not just noisy) — e.g. a lidar
  return pattern consistent with dust/fog/glass rather than real geometry, or
  an IMU saturated by vibration reporting a physically impossible attitude
  rate. Excluded from fusion; logged.
- **Dead** — no data, or data rejected for long enough that the sensor is
  treated as absent. Triggers the next rung of [[Sensor Fallbacks]].

## Detecting a lying sensor

Two complementary techniques:

1. **Cross-checking sources.** The same displacement/attitude estimate is
   available from more than one source once the pipeline is layered (lidar
   odometry, the [[Motor Physics Prior]], and eventually RGB features). A
   sensor whose output disagrees with the consensus of the others, repeatedly
   and beyond the expected error margin, is the suspect.
2. **Revisiting known places.** Compare current sensing against the
   [[Persistent Map]] when the drone returns to (or nearly to) a place it has
   already mapped — a form of place recognition. If the current scan does not
   match what the map says should be there, either the drone's pose estimate
   has drifted (an odometry problem) or the sensor producing the current scan
   is unreliable (a sensing problem); disambiguating the two is itself part
   of the online error model below.

## Modelling sensor error online

Rather than a fixed noise model, track each sensor's *measured* disagreement
against the consensus/place-recognition checks over time, and feed that back
as a live covariance/weight in the fusion. A sensor that is usually right but
occasionally noisy should be down-weighted proportionally, not treated as
binary healthy/dead. This is the piece that turns "detect a lying sensor" into
"know how much to trust it right now."

## Livox-specific failure modes

- **Dust, glass, direct sun** — dust and glass are classic lidar failure
  surfaces (spurious or missing returns); direct sun can saturate the
  receiver or inject noise. Detectable partly via return-intensity
  statistics, partly via disagreement with the physics prior.
- **Degenerate geometry** — long corridors (motion along the corridor axis is
  underconstrained) and open fields (too few features to register against) are
  classic LIO degeneracy cases. The maze structure in [[Phase 4 Maze Mission]]
  is exactly the geometry where this needs to be watched, both for corridor
  legs and for the open area around the structure.
- **IMU saturation/vibration** — the Mid-360's built-in IMU (and the Pixhawk's)
  can saturate or report garbage under strong vibration, especially close to
  the props on an X8. Cross-check against the physics prior, which does not
  depend on either IMU being clean.

## Status

Design only — no implementation yet. Depends on [[LIO Odometry]] and
[[Persistent Map]] existing first, since both cross-checking and place
recognition need a working baseline to compare against.
