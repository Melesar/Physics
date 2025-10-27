# Inspector window

The goal is to implement an inspector window for the edit mode. The inspector will be used to:

1) Edit the properties of the selected object
2) Create new objects
3) Delete existing objects

## How it should work

- When the user enters the edit mode the inspector window should appear on the rigth side of the screen.
- When nothing is selected, the window should display options to create a new object
  * First should be a dropdown to select the type of the object. For now only a cylinder and a sphere
  * Depending on the selected type, different input fields should appear next.
  * Height and radius for cylinder and just the radius for the sphere
  * After object parameters there should be a `Create` button.
  * `Create` button should create the configured object at the `(0, 0, 0)` position and unrotated
- When an object is selected, the window should display and allow to edit its properties:
  * Position
  * Rotation (as an Euler angles vector - not as a quaternion)
  * Mass of the object
- At the bottom of the window there should be a `Delete object` button, which should remove it, asking for confirmation first.
- The user can select an object only during the edit mode by clicking on it with a mouse.
- If the user clicks on a different object, it should become the current selection.
- When the user presses `ECS` during the edit mode, it should unselect the currently selected object if there is one.
- The gizmos should appear only for the selected object.

## Technical requirements

First, to reduce the maintenance burden, we'll focus solely on the `rigidbodies` scenario. Other scenarios must be moved outside the `scenarios` directory into a new one called `legacy`. It will exist in the project root, therefore these scenarios will not be picked up by the build system anymore.

1) In the `core.h` define a new function `object* select_object(Ray ray)`. This function should be implemented in all scenarios (for now only the `rigidbodies` will have a non-default implementation)
2) The function should check the intersection of the provided ray with all the objects in the scenario and return a pointer to the one which passed the test. If none passed, return `NULL`. Use `GetRayCollisionMesh` function from Raylib
3) This function should be called from `main.c` in the game loop when edit mode is active. `main.c` should also manage the selected object and the inspector window
4) Define the enum in the `core.h`: `enum object_type { cylinder, sphere }`.
5) Another two functions that should be defined in `core.h` and implemented in scenarios are:
  * `void create_object(object_type type, Vector3 params)`. This should create a specified object inside the scenario.
    - For cylinders: `params.x` is height, `params.y` is radius
    - For spheres: `params.x` is radius
  * `void delete_object(object *obj)`. Don't implment this for now, just declare a function with an empty body in the scenario.
  * These two function should be called by `main.c` when the user presses the corresponding buttons in the inspector window.
  
