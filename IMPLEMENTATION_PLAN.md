# Dark Mode Visual Implementation Plan

## Overview
This plan details the implementation of a "dark mode" visual style for the physics simulation testbed. The goal is to achieve a high-contrast, technical aesthetic with proper lighting, fog, and materials as specified in `shading_reference.md`.

## Current State Analysis

### Existing Implementation (core/main.c)
- **Background**: RAYWHITE (line 135) - needs to change to #121214
- **Ground Plane**: LIGHTGRAY (line 144) - needs to change to #080808 with PBR properties
- **Grid**: Standard DrawGrid (line 145) - needs custom colors (#444444, #222222)
- **Lighting**: Single directional light + ambient - needs 3-point setup
- **Shader**: Using vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.vs/fs
- **No fog implementation** - needs linear fog
- **No wireframe overlays** - needs implementation
- **No color palette system** - needs implementation

### Available Resources
- Raylib shader examples in `vendor/raylib/examples/shaders/resources/shaders/glsl330/`
- Existing lighting system using rlights.h
- Scenarios: pendulum.c, rigidbodies.c, rope.c, spring.c

---

## Stage 1: Environment Setup (Background & Fog)

### 1.1 Background Color
**File**: `core/main.c:135`

**Changes**:
```c
// Replace:
ClearBackground(RAYWHITE);
// With:
ClearBackground((Color){0x12, 0x12, 0x14, 0xFF}); // #121214
```

**Verification**: Build and run - background should be near-black with slight cool tint.

---

### 1.2 Fog Implementation
**Approach**: Modify the existing lighting shader to include fog calculations.

**Option A**: Extend existing lighting.fs shader
- Copy `vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.fs` to `shaders/lighting_fog.fs`
- Copy `vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.vs` to `shaders/lighting_fog.vs`
- Reference `fog.fs` (lines 39, 88-89) for fog implementation pattern

**Option B**: Use fog.fs as base and add lighting
- More complex, not recommended

**Recommended**: Option A

**Changes to lighting_fog.fs**:
1. Add uniform variables:
   ```glsl
   uniform vec3 fogColor;      // Will be set to #121214
   uniform float fogStart;     // 20.0
   uniform float fogEnd;       // 100.0
   ```

2. In main() function, after lighting calculations:
   ```glsl
   // Calculate distance from camera to fragment
   float dist = length(viewPos - fragPosition);

   // Calculate linear fog factor
   float fogFactor = clamp((fogEnd - dist) / (fogEnd - fogStart), 0.0, 1.0);

   // Mix final color with fog color
   finalColor.rgb = mix(fogColor, finalColor.rgb, fogFactor);
   ```

**Changes to main.c**:
1. Update shader paths (lines 196-197):
   ```c
   char *vs_shader_path = "shaders/lighting_fog.vs";
   char *fs_shader_path = "shaders/lighting_fog.fs";
   ```

2. After loading shader, set fog uniforms:
   ```c
   int fogColorLoc = GetShaderLocation(shader, "fogColor");
   int fogStartLoc = GetShaderLocation(shader, "fogStart");
   int fogEndLoc = GetShaderLocation(shader, "fogEnd");

   SetShaderValue(shader, fogColorLoc, (float[3]){0x12/255.0f, 0x12/255.0f, 0x14/255.0f}, SHADER_UNIFORM_VEC3);
   SetShaderValue(shader, fogStartLoc, (float[1]){20.0f}, SHADER_UNIFORM_FLOAT);
   SetShaderValue(shader, fogEndLoc, (float[1]){100.0f}, SHADER_UNIFORM_FLOAT);
   ```

**Verification**: Distant objects should fade into the background color.

---

## Stage 2: Lighting System Update

### 2.1 Three-Point Lighting Setup
**File**: `core/main.c`, function `setup_lighting()` (lines 195-208)

**Current**: One directional light at (-2, 5, -2)

**Target**:
- Ambient: #404040, intensity 1.0
- Directional (Key): #ffffff, intensity 2.5, position (10, 20, 10)
- Point (Rim): #4444ff, intensity 500.0, position (-10, 10, -10)

**Changes**:
```c
static Shader setup_lighting() {
  char *vs_shader_path = "shaders/lighting_fog.vs";
  char *fs_shader_path = "shaders/lighting_fog.fs";

  Shader shader = LoadShader(vs_shader_path, fs_shader_path);

  // Key light (Directional) - Main shadow caster
  Light keyLight = CreateLight(
    LIGHT_DIRECTIONAL,
    (Vector3){10.0f, 20.0f, 10.0f},
    Vector3Zero(),
    WHITE,  // #ffffff
    shader
  );
  keyLight.enabled = 1;
  UpdateLightValues(shader, keyLight);

  // Rim light (Point) - Blue highlight
  Light rimLight = CreateLight(
    LIGHT_POINT,
    (Vector3){-10.0f, 10.0f, -10.0f},
    Vector3Zero(),
    (Color){0x44, 0x44, 0xff, 0xff},  // #4444ff
    shader
  );
  rimLight.enabled = 1;
  UpdateLightValues(shader, rimLight);

  // Ambient light
  int ambientLoc = GetShaderLocation(shader, "ambient");
  SetShaderValue(shader, ambientLoc, (float[4]){0x40/255.0f, 0x40/255.0f, 0x40/255.0f, 1.0f}, SHADER_UNIFORM_VEC4);

  // Set fog parameters
  int fogColorLoc = GetShaderLocation(shader, "fogColor");
  int fogStartLoc = GetShaderLocation(shader, "fogStart");
  int fogEndLoc = GetShaderLocation(shader, "fogEnd");

  SetShaderValue(shader, fogColorLoc, (float[3]){0x12/255.0f, 0x12/255.0f, 0x14/255.0f}, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader, fogStartLoc, (float[1]){20.0f}, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader, fogEndLoc, (float[1]){100.0f}, SHADER_UNIFORM_FLOAT);

  return shader;
}
```

**Note**: Check rlights.h to ensure it supports multiple lights. If MAX_LIGHTS is insufficient, may need to increase it or modify the shader.

**Light Intensity Handling**:
- The directional light intensity of 2.5x might need to be handled in shader or by adjusting the light color values
- Point light "intensity 500.0" likely refers to attenuation distance; verify in rlights.h CreateLight implementation

**Verification**: Objects should be well-lit from above-right with subtle blue rim lighting from behind-left.

---

## Stage 3: Ground Plane & Grid Customization

### 3.1 Ground Plane Material
**File**: `core/main.c:144`

**Current**:
```c
DrawPlane((Vector3){0.0f, 0.0f, 0.0f}, (Vector2){32.0f, 32.0f}, LIGHTGRAY);
```

**Target Properties**:
- Color: #080808
- Roughness: 0.8
- Metalness: 0.2
- Size: 200x200 units (currently 32x32)

**Problem**: Raylib's DrawPlane doesn't support PBR material properties directly.

**Solution**: Need to create a custom ground plane with material:

**Changes**:
1. At file level (after includes), add ground mesh and material:
   ```c
   static Model groundModel;
   static bool groundInitialized = false;
   ```

2. Create initialization function (add after setup_lighting):
   ```c
   static void setup_ground_plane(Shader shader) {
     // Create plane mesh
     Mesh mesh = GenMeshPlane(200.0f, 200.0f, 1, 1);
     groundModel = LoadModelFromMesh(mesh);

     // Set shader
     groundModel.materials[0].shader = shader;

     // Set color
     groundModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = (Color){0x08, 0x08, 0x08, 0xFF};

     // Note: Raylib's standard material doesn't have explicit roughness/metalness
     // These would need to be handled in a PBR shader
     // For now, the dark color will be the primary visual change

     groundInitialized = true;
   }
   ```

3. Call in main() after setup_lighting (line ~63):
   ```c
   setup_ground_plane(shader);
   ```

4. Modify draw_scene to use the model (line 144):
   ```c
   // Replace DrawPlane line with:
   if (groundInitialized) {
     DrawModel(groundModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
   }
   ```

**Advanced PBR Option** (if needed):
- Use pbr.fs/vs shaders from vendor/raylib/examples/shaders/resources/shaders/glsl330/
- Would require more complex material setup with roughness and metalness textures or uniforms

**Verification**: Ground should be nearly black (#080808) instead of light gray.

---

### 3.2 Custom Grid Colors
**File**: `core/main.c:145`

**Current**:
```c
DrawGrid(32, 1.0f);  // Uses default colors
```

**Target**:
- Main division color: #444444
- Sub-division color: #222222

**Problem**: DrawGrid uses hardcoded colors in Raylib.

**Solution Options**:

**Option A**: Draw custom grid lines
```c
static void draw_custom_grid(int slices, float spacing) {
  int halfSlices = slices / 2;
  Color mainColor = (Color){0x44, 0x44, 0x44, 0xFF};    // Main divisions
  Color subColor = (Color){0x22, 0x22, 0x22, 0xFF};     // Sub divisions

  for (int i = -halfSlices; i <= halfSlices; i++) {
    Color lineColor = (i % 10 == 0) ? mainColor : subColor;

    // Lines parallel to Z axis
    DrawLine3D(
      (Vector3){i * spacing, 0.01f, -halfSlices * spacing},
      (Vector3){i * spacing, 0.01f, halfSlices * spacing},
      lineColor
    );

    // Lines parallel to X axis
    DrawLine3D(
      (Vector3){-halfSlices * spacing, 0.01f, i * spacing},
      (Vector3){halfSlices * spacing, 0.01f, i * spacing},
      lineColor
    );
  }
}
```

**Option B**: Use rlgl to draw lines (more efficient)
- Would require including rlgl.h
- Use rlBegin(RL_LINES), rlVertex3f, rlEnd pattern

**Recommended**: Option A for simplicity, Option B for performance if needed.

**Changes to main.c:145**:
```c
// Replace:
DrawGrid(32, 1.0f);
// With:
draw_custom_grid(32, 1.0f);
```

**Note**: Grid is drawn at y=0.01 to prevent Z-fighting with ground plane (as per spec).

**Verification**: Grid should have dark gray main lines and darker gray subdivision lines.

---

## Stage 4: Physics Objects Material System

### 4.1 Color Palette Definition
**File**: Create new file `include/colors.h`

**Content**:
```c
#ifndef COLORS_H
#define COLORS_H

#include "raylib.h"

// Dark mode color palette for physics objects
#define COLOR_GREEN_ACTIVE   (Color){0x00, 0xff, 0x88, 0xFF}  // Active/Standard
#define COLOR_RED_HIGHLIGHT  (Color){0xff, 0x33, 0x66, 0xFF}  // Highlight/Alert
#define COLOR_BLUE_STATIC    (Color){0x33, 0x66, 0xff, 0xFF}  // Static/Passive
#define COLOR_YELLOW_INFO    (Color){0xff, 0xcc, 0x00, 0xFF}  // Info/Sensor

// Background and environment
#define COLOR_BACKGROUND     (Color){0x12, 0x12, 0x14, 0xFF}
#define COLOR_GROUND         (Color){0x08, 0x08, 0x08, 0xFF}
#define COLOR_GRID_MAIN      (Color){0x44, 0x44, 0x44, 0xFF}
#define COLOR_GRID_SUB       (Color){0x22, 0x22, 0x22, 0xFF}

#endif // COLORS_H
```

**Changes to main.c**:
Add include at top:
```c
#include "colors.h"
```

Update color references to use these constants.

**Verification**: Compile successfully with new header.

---

### 4.2 Material Properties for Objects
**Files**: scenarios/rigidbodies.c

**Current State**: Unknown - need to check how objects are currently drawn.

**Target Properties** (per spec):
- Roughness: 0.3 (semi-smooth, specular highlights)
- Metalness: 0.7 (metallic look)
- Flat shading: Enabled (optional)

**Approach**:
1. If scenarios use DrawCube, DrawSphere, DrawCylinder directly:
   - These need to be replaced with Model-based drawing
   - Models allow material property control

2. If scenarios already use Models:
   - Update material properties

**Implementation Pattern** (example for a cube):
```c
// In setup/initialization:
Model cubeModel = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
cubeModel.materials[0].shader = shader;  // Use scene shader
cubeModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = COLOR_GREEN_ACTIVE;

// For PBR properties, if using pbr.fs shader:
// Would need to set additional uniforms for roughness, metalness
```

**Verification**: Objects should have the correct colors from the palette.

---

## Stage 5: Wireframe Overlay Implementation

### 5.1 Wireframe Rendering Approach
**Target**: White wireframe (#ffffff) at 10% opacity overlaid on solid objects.

**Approach Options**:

**Option A**: Two-pass rendering per object
1. First pass: Draw solid mesh with material
2. Second pass: Draw wireframe with transparency

**Option B**: Wireframe-only shader with transparency
- Draw wireframe as separate geometry

**Option C**: Geometry shader (if supported)
- Generate wireframe from geometry shader

**Recommended**: Option A (two-pass) for maximum compatibility.

**Implementation**:

1. **Enable wireframe mode in Raylib**: Use rlEnableWireMode() / rlDisableWireMode()

2. **Drawing pattern** (in scenario draw functions):
   ```c
   // Draw solid object
   DrawModel(objectModel, position, scale, WHITE);

   // Draw wireframe overlay
   rlDisableDepthTest();  // Draw over solid
   rlEnableWireMode();
   DrawModel(objectModel, position, scale, (Color){255, 255, 255, 25});  // ~10% opacity
   rlDisableWireMode();
   rlEnableDepthTest();
   ```

3. **Blending setup**: Ensure blending is enabled for transparency:
   ```c
   // In initialization:
   rlSetBlendMode(BLEND_ALPHA);
   ```

**Note**: This requires modifying each object's draw call in scenario files.

**Alternative**: Create a helper function:
```c
void draw_object_with_wireframe(Model model, Vector3 position, float scale, Color color) {
  // Draw solid
  model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = color;
  DrawModel(model, position, scale, WHITE);

  // Draw wireframe
  rlEnableWireMode();
  DrawModel(model, position, scale, (Color){255, 255, 255, 25});
  rlDisableWireMode();
}
```

**Verification**: Objects should have subtle white wireframe edges visible over solid color.

---

## Stage 6: Integration & Testing

### 6.1 Scenario File Updates
**Files**: scenarios/rigidbodies.c, scenarios/pendulum.c, etc.

**Changes needed per scenario**:
1. Include colors.h
2. Update object initialization to use Models with materials
3. Update draw calls to use wireframe overlay technique
4. Assign appropriate colors from palette

**Pattern**:
```c
#include "colors.h"

// In setup:
Model myObject = LoadModelFromMesh(GenMeshSphere(1.0f, 32, 32));
myObject.materials[0].shader = shader;  // Passed from main
myObject.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = COLOR_RED_HIGHLIGHT;

// In draw:
draw_object_with_wireframe(myObject, position, 1.0f, COLOR_RED_HIGHLIGHT);
```

---

### 6.2 Build & Visual Verification Checklist

**Environment**:
- [ ] Background is #121214 (dark with cool tint)
- [ ] Fog fades objects into background at 20-100 units
- [ ] Ground plane is #080808 (nearly black)
- [ ] Grid has correct colors (#444444 main, #222222 sub)

**Lighting**:
- [ ] Ambient light is subtle (#404040)
- [ ] Key light creates clear shadows from upper-right
- [ ] Rim light adds blue highlights from back-left
- [ ] Objects are clearly defined by shadows

**Objects**:
- [ ] Objects use palette colors (green, red, blue, yellow)
- [ ] Objects have semi-smooth metallic appearance
- [ ] Wireframe overlay is visible but subtle (10% white)
- [ ] Shadows cast correctly on ground plane

**Performance**:
- [ ] Maintains 60 FPS at 1920x1080
- [ ] No Z-fighting between ground and grid
- [ ] Fog calculation doesn't impact performance significantly

---

## Stage 7: Refinement & Polish

### 7.1 Potential Issues & Solutions

**Issue**: Light intensity not matching spec
- **Solution**: Adjust light color values multiplicatively (e.g., key light color * 2.5)

**Issue**: PBR properties not visible with basic lighting shader
- **Solution**: Switch to pbr.fs/vs shaders and set up proper uniforms

**Issue**: Wireframe too visible or not visible enough
- **Solution**: Adjust opacity value (currently 25/255, can range 15-40)

**Issue**: Fog too strong or too weak
- **Solution**: Adjust fogStart/fogEnd values

**Issue**: Performance degradation from double rendering
- **Solution**: Consider single-pass wireframe shader or reduce wireframe detail

---

### 7.2 Optional Enhancements

**Flat Shading** (if needed for visual clarity):
- Modify vertex shader to not interpolate normals
- Or use face normals instead of vertex normals in fragment shader

**Shadow Mapping** (if soft shadows needed):
- Spec mentions "PCF Soft Shadow Maps"
- Would require implementing shadow map rendering pass
- Reference: shadowmap.fs/vs in shader examples

**Color-coded Depth** (for debugging):
- Add shader uniform to toggle depth visualization
- Useful for debugging fog and depth issues

---

## Implementation Order Summary

1. **Stage 1**: Background color + fog shader modifications
2. **Stage 2**: Three-point lighting setup
3. **Stage 3**: Ground plane material + custom grid
4. **Stage 4**: Color palette + object materials
5. **Stage 5**: Wireframe overlay system
6. **Stage 6**: Scenario integration + testing
7. **Stage 7**: Refinement based on visual comparison with reference image

---

## Key Files to Create/Modify

### New Files:
- `shaders/lighting_fog.vs` (copy + modify)
- `shaders/lighting_fog.fs` (copy + modify)
- `include/colors.h` (new)

### Modified Files:
- `core/main.c` (primary changes)
- `scenarios/rigidbodies.c` (material updates)

---

## Technical Considerations

### Shader Compatibility
- Target: GLSL 330 (OpenGL 3.3)
- Ensure shader code is compatible with Raylib's shader system
- Test on target platform (Linux)

### Color Space
- Spec recommends "linear workflow"
- Raylib may use sRGB by default
- May need gamma correction in shader if colors appear incorrect

### Performance Budget
- Target: 60 FPS @ 1920x1080
- Most expensive operations: Fog calculation, dual-pass wireframe
- Profile if performance issues arise

### Raylib API Usage
- rlights.h for lighting (already in use)
- Check MAX_LIGHTS constant (may need adjustment for 3 lights)
- Verify CreateLight supports intensity multipliers

---

## Fallback Plans

**If fog shader modification is too complex**:
- Use post-process fog (blend screen with fog color based on depth)
- Simpler but less accurate

**If three-point lighting exceeds MAX_LIGHTS**:
- Combine key + rim into single light with averaged properties
- Or increase MAX_LIGHTS in rlights.h

**If PBR materials needed but too complex**:
- Use basic Phong shading with adjusted parameters
- Visual result may differ but will be functional

**If wireframe overlay impacts performance**:
- Make it toggleable with a key
- Or reduce to outline-only (edge detection shader)

---

## Testing Strategy

1. **Incremental**: Test each stage before proceeding
2. **Visual Comparison**: Compare against reference image after each stage
3. **Performance**: Monitor FPS throughout
4. **Cross-scenario**: Test all scenarios to ensure consistency

---

## Success Criteria

The implementation is complete when:
1. Visual output closely matches reference image (image.jpg)
2. All technical specifications from shading_reference.md are implemented
3. Performance target of 60 FPS is maintained
4. All scenarios render correctly with new visual style
5. Code is clean, well-commented, and maintainable
