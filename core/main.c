#include "raylib.h"

void setup();

void simulate(float dt);

void draw();

int main(void)
{
    // Initialization
    const int screenWidth = 1920;
    const int screenHeight = 1080;

    InitWindow(screenWidth, screenHeight, "Raylib Prototype - Cube and Ground");

    // Define the camera to look into our 3D world
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 4.0f, 4.0f, 4.0f }; // Camera position
    camera.target = (Vector3){ 0.0f, 1.0f, 0.0f };   // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };       // Camera up vector
    camera.fovy = 45.0f;                             // Field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;          // Perspective projection

    SetTargetFPS(60); // Set frame rate

    setup();

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        UpdateCamera(&camera, CAMERA_FREE); // Move camera around using mouse + WASD

        simulate(GetFrameTime());

        // Draw
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                draw();

                // Draw ground plane
                DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 32.0f, 32.0f }, LIGHTGRAY);
                DrawGrid(32, 1.0f); // Helper grid

            EndMode3D();

            DrawText("Move with WASD + Mouse. Press ESC to quit.", 10, 10, 20, DARKGRAY);

        EndDrawing();
    }

    // De-Initialization
    CloseWindow(); // Close window and OpenGL context

    return 0;
}


