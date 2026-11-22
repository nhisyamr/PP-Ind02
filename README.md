# Proyek Perancangan IND-02 — ASRS WMS & 3D Digital Twin

Industrial Web HMI for the **IND-02 3-Axis Belt-Driven Mini ASRS** featuring a live 2.5D photorealistic CAD Digital Twin (`A1`–`C3`), Auto FIFO / Manual allocation, ESP32 Dual-Core Firebase RTDB telemetry sync, and commissioning controls.

## Repository Contents

- `index.html` — Main WMS Dashboard with integrated 3D CAD Digital Twin (`Split`, `3D Twin`, and `Matrix` modes)
- `asrs_cad_widget/asrs-cad-twin.js` — Zero-dependency HTML5 Canvas Digital Twin & 60 FPS kinematic smoothing engine
- `asrs_cad_widget/assets/` — 16 pixel-locked PNGs (`1200x896` all-empty rack backdrop, gantry/box sprites, and 9-bay 3D occlusion masks)
- `.nojekyll` — Disables Jekyll preprocessing on GitHub Pages for fast static deployment

## Deploy to GitHub Pages

1. Upload or push all files in this directory to your repository root (`main` branch).
2. In GitHub, open **Settings → Pages → Build and deployment** and select **Deploy from a branch** (`main` / `/ (root)`).
3. Wait 1–2 minutes for the green check in the **Actions** tab, then open your GitHub Pages URL and hard-refresh (`Ctrl+Shift+R` / `Cmd+Shift+R`).
