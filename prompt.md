# Implement a new scenario for simplified collision testin

How to implement a scenario:

1) Create a `.c` file in the scenarios directory. It's name will be the scenario's name
```
Example: scenario "collisions" -> collisions.c file
```
2) Implement the functions from the `include/core.h` file. Those are the functions which follow the struct definitions and grouped together. `draw_x` functions are implemented already.
3) Add an entry to the `.gitignore` file. The entry corresponds to the scenario's binary which will be generated in the project's root.

## Scenario's main functions

**init_program**: Here we can setup the window's name and camera parameters
**setup_scene**: Initialize all the data needed for the simulation
**process_inputs**: Capture the user's inputs and adjust the simulation accordingly
**simulate**: Run the simulation step
**draw**: Render the scene
**draw_ui**: Draw the UI for debugging purposes

## The scenario

The main goal of the scenario to implement gizmos that would allow to move and rotate the cylinder, so that we can test it's collisions with the plane. Here is the plan:

1) Create a cylinder similar to the `rigidbodies` scenario. @scenarios/rigidbodies.c
2) Render gizmos arond the cylinder for translation and rotation. For translation you may use `draw_arrow` function. For rotation you will need to implement your own.
3) Allow dragging these gizmos with a mouse to move and rotate the cylinder around respective axis. This is similar to what every game engine or 3D software does.
4) In the `simulate` function, run the collision detection of the cylinder with the ground plane. When the collision is detected, change the cylinder's color to green, otherwise make it blue.

In this scenario, the cylinder will not be the subject of the physics simulation. It would move only in response to the player input. The only goal is to test collision detection.
