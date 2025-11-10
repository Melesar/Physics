# Physics Testbed Visual Specification
## Overview

This document specifies the visual design for a "Dark Mode" physics simulation testbed. The aesthetic aims for high contrast and clarity suitable for debugging, using a technical, slightly stylized appearance reminiscent of CAD or synthesizer visualizations.

## Core Tech Stack (Reference Implementation)
 * Color Space: Linear workflow recommended for accurate lighting.
 * Shadows: PCF Soft Shadow Maps (or equivalent filtered shadow technique).

## 1. Scene & Environment
The environment is designed to be non-distracting while providing necessary depth cues.
 * Background Color: #121214 (Near-black with a slight cool/purple tint to avoid pure void black).
 * Fog: Linear fog matching the background color #121214.
   * Start Distance: 20 units
   * End Distance: 100 units
   * Purpose: Fades distant geometry into the background, hiding render distance limits and adding depth.

## 2. Lighting Setup
A three-point-like setup is used to ensure objects are visible from all angles but clearly defined by shadows.
| Light Type | Color | Intensity | Position (x,y,z) | Notes |
|---|---|---|---|---|
| Ambient | #404040 | 1.0 | N/A | Base illumination to prevent pitch-black shadows. |
| Directional (Key) | #ffffff | 2.5 | (10, 20, 10) | Main shadow caster. High angle to minimize long shadows. |
| Point (Rim) | #4444ff | 500.0 (decay) | (-10, 10, -10) | Adds a subtle blue highlight to opposite edges, defining shape. |

## 3. Ground Plane & Grid
The ground provides spatial context without visual noise.
 * Ground Geometry: Large plane (e.g., 200x200 units).
 * Ground Material: Standard PBR Material.
   * Color: #080808 (Almost pure black).
   * Roughness: 0.8 (Matte, diffuses light slightly).
   * Metalness: 0.2 (Slight reflective quality).
 * Grid Overlay:
   * Lines: Standard debug grid drawn slightly above ground (y=0.01) to avoid Z-fighting.
   * Main Division Color: #444444
   * Sub-division Color: #222222

## 4. Physics Objects Styling
Objects use a composite rendering technique: a solid base mesh for mass/lighting, overlaid with a faint wireframe for technical clarity.

A. Base Mesh (Solid)
Standard PBR material that reacts to light and casts/receives shadows.
 * Roughness: 0.3 (Semi-smooth, allows for specular highlights to show rotation).
 * Metalness: 0.7 (Metallic look, increases contrast of reflections).
 * Flat Shading: Enabled (optional, but helps visualize low-poly rotation better than smooth shading).

B. Wireframe Overlay
A separate render pass or child object using actual line geometry (not a texture).
 * Type: LineSegments using WireframeGeometry (renders all edges, including internal triangulations if desired, though mostly just sharp edges is preferred for cleanliness).
 * Color: #ffffff (White).
 * Opacity: 0.1 (10% visible). Must be transparent to just barely highlight the structure without obscuring the solid color.

C. Color Palette (Reference)
Distinct, high-saturation neon-like colors to stand out against the dark background.
 * Green (Active/Standard): #00ff88
 * Red/Pink (Highlight/Alert): #ff3366
 * Blue (Static/Passive): #3366ff
 * Yellow (Info/Sensor): #ffcc00
