# Implement camera movement

The task is to implment a more convenient camera movement than what Raylib has to offer.

## Requirements:

- Camera should move forward, backward, left and right with WSAD keys respectively.
- It should rotate with the mouse while holding the RMB
- Movement speed and rotation sensitivity should be adjustable via UI

## Implementation hints

- Put the code into `main.c`. Implement it as a function that is called each frame.
- To actually update the camera use Raylib's `void UpdateCameraPro(Camera *camera, Vector3 movement, Vector3 rotation, float zoom)` function. It's available in the `rcamera.h` header like so:

```
#define RCAMERA_IMPLEMENTATION
#include "rcamera.h"
```

Here is some more info on this function:

```
    // Required values
    // movement.x - Move forward/backward
    // movement.y - Move right/left
    // movement.z - Move up/down
    // rotation.x - yaw
    // rotation.y - pitch
    // rotation.z - roll
    // zoom - Move towards target
```

So the the process will look like the following:
  - You call a function at the beginning of every frame
  - You gather user's inputs
  - You calculate the movement required
  - You update the camera with `UpdateCameraPro`

- The UI to adjust camera's settings implement as a new Nuklear window in the top-right corner of the screen. For the reference on how to use Nuklear, you may refer to the existing scenarios in the `scenarios` directory.



