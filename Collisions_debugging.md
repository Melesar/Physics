# Physics collisions debugging

## Brief feature description

We need to implement a collision debugging mode. In this mode, the simulation will be paused after a certain collision occurs and we will be able to step through each step of penetration resolution and velocity resolution.
Additionaly, all corrections will be shown in the scene with arrows and displayed in the UI widget.

## Collision resolution anatomy

**Step 0: Collision detection**

This produces a list of _collisions_ and _contacts_. Each contact is assigned to a collision and each collision can have multiple contacts. A collision occurs only between two bodies.

A collision can be _dynamic_ or _static_. 
- Dynamic collision means that both bodies involved are dynamic, i.e. have finite mass and can move around. 
- Static collision means that one body is static. It's treated as immovable with infinite mass.

Static bodies do not generate collisions between each other.

The two following steps run in a pattern:

1) Find the worst contact according to some metric.
2) Adjust the bodies involved in the respective collision.
3) Update the internal state of all the other contacts which might have been affected.

These steps are repeated until either:
- There is a contact _at least_ as bad as some threshold.
- Maximum number of attemps has been reached.

**Step 1: Penetration resolution**

- **Contact search criteria:** penetration depth. This is denoted by `contact->depth` field in the code.
- **Bodies correction:** position and rotation. The goal is to separate the bodies in contact so that they don't touch anymore.
- **Contacts update:** depth value. After moving and rotating the two bodies, penetration depths of other contacts might have changed.
- **Source:** `physics.c`
```
resolve_interpenetration_contact(world, collision_index, contact, deltas);
update_penetration_depths(world, collision_index, deltas);
```

**Step 2: Velocity resolution**

- **Contact search criteria:** desired velocity change. `contact->desired_delta_velocity` in code.
- **Bodies correction:** linear velocity and angular momentum. The goal is to have the bodies bounce off one another and move in opposite directions.
- **Contacts update:** `contact->desired_delta_velocity`
- **Source:** `physics.c`
```
resolve_velocity_contact(world, worst_collision_index, contact, deltas);
update_velocity_deltas(world, worst_collision_index, deltas, dt);
```

## Entry point: UI

The feature flow starts with a checkbox in the widget controls. If it's enabled, the new widget appears and also the simulation turns into debugging mode.

When a collision is detected, the widget displays the key information about it in the header:
- Current iteration count.
- The state of the contact being resolved.
- Collision type: dynamic or static.

The widget's body should change depending on the current phase of collision resolution.

**1) Penetration resolution**

Shows deltas applied to both bodies like so:
```
Deltas:

       Body 1               Body 2
Linear:  (0, 1, 0)   |  Linear:  (1, 0, 0)
Angular: (0, -1, 0)  |  Angular: (-1, 0, 0)
```

If the collision is static, only display the first column.

**2) Depth updates**

Shows the updates to the depths of contacts like so:

```
Contact #1: 0.1 -> 0
Contact #2: 0.2 -> 0.07
Contact #3: 0.01 -> 0.15
```

Only show non-zero updates. For numbering, use indices in the contacts array of the `collisions` struct.

**3) Velocities resolution**

Show velocity deltas similar to the penetration case.

**4) Desired velocity updates**

Shows updates to the contact velocities like so:

```
Contact #1:
  Local velocity: (0.2, 1, 3) -> (0, 0, 0)
  Desired delta: 0.1 -> 0
Contact #2:
  Local velocity: (0.4, 2, 2.1) -> (0.8, 1.4, 0)
  Desired delta: 0.5 -> 0.1
```

Only show non-zero updates. For numbering, use indices in the contacts array of the `collisions` struct.

## Implementation hints

- Reuse as much code as possible
- When the debugging mode is enabled, the simulation should run normally until there is at least one _dynamic_ collision.
- Once it happens, the simulation should pause and advance step-by-step, stopping at every resolution iteration in every phase. See `toggle_pause` function.
- There are key functions representing each step:
  - `resolve_interpenetration_contact`
  - `update_penetration_depths`
  - `resolve_velocity_contact`
  - `update_velocity_deltas` / `update_desired_velocity_delta`
  Make these functions return the required data for debugging, so that they can be used by the normal flow as well.
- For debugging, create another physics entry point called `physics_step_debug`. It should reuse the functional code from the normal flow while simultaneousely handling stopping the simulation, gathering data for display and advancing one step at a time.
