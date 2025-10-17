# Physics simulations

The project's goal is to incrementally built physics simulations to ultimately arrive
at building custom ragdolls.

## Project structure

- **Scenarios**: these are specific simulation scenarios described as `.c` files in the `scenarios` directory. Each of them gets built into a separate executable.
- **Core files**: these file contains common logic and scene setup for all scenarios. They reside in the `core` directory.
- **main.c**: The `core/main.c` file contains project setup and a scene template that is common for all scenarios. There is also an update loop with function declarations that each scenario is supposed to implement.


