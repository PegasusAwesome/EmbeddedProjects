# Rounded-heart health tracker

Ten removable heart pins and a rectangular tray, designed for PLA on an FDM printer. Each heart and its peg are one solid print. The tray is a separate print. All STL coordinates are **millimeters**; import at **100% scale**.

## Files to print

| File | Quantity | Description |
| --- | --- | --- |
| `heart_pin.stl` | 10 | One rounded heart with its square peg, already lying flat on the bed. |
| `hearts_10_flat.stl` | 1 instead of the single-heart file | Ten separate heart pins arranged in two rows of five. |
| `tray_10_hearts.stl` | 1 | Full tray with ten 3.6 mm square sockets, already bottom down. |
| `peg_fit_test.stl` | 1 initially | Four sockets labeled with their widths: 3.4, 3.5, 3.6, and 3.7 mm. |

Use either ten copies of the single heart or the ten-heart layout, not both.

## Check the fit first

Print **one heart and the fit-test strip** with the same material and settings you will use for the final parts. Let them cool and try each socket. Choose the smallest size that inserts and removes comfortably by hand; the hearts should remain easy to remove during play.

The main tray uses the **3.6 mm** socket. If another size fits better, use the corresponding tray STL in `tray-fit-options/`. Those are complete replacement trays with 3.4, 3.5, or 3.7 mm sockets. Do not scale the whole tray to adjust the fit.

## Starting slicer settings

- Material: PLA, using your printer's normal PLA profile.
- Nozzle: 0.4 mm; layer height: 0.16 or 0.20 mm.
- Walls: 3; top and bottom thickness: at least 0.8 mm.
- Hearts: 100% infill. Tray and fit strip: 15–20% infill.
- Supports: off. No raft. A brim is optional if the small heart pins lift from the bed.
- Keep the supplied orientations: hearts flat, tray and fit-strip sockets facing up.
- Print the hearts in red and the tray in a contrasting color, on separate plates if needed. No multi-color printer is required.

## Dimensions

- Tray: **150 × 28 × 10 mm**.
- Heart silhouette: **12 mm wide**, approximately **12 mm tall**, **3.2 mm thick**.
- Complete pin lying flat: **12 × 19.5 × 3.2 mm**, including its peg.
- Peg: **3.2 × 3.2 mm** square; extends 7.5 mm below the nominal heart tip and blends into the heart for strength.
- Standard socket: **3.6 × 3.6 mm**, **8 mm deep**, with a small entrance chamfer; 2 mm of material remains below it.
- Nominal side clearance: **0.2 mm**; hearts are spaced **14 mm** center to center.

The pins rest against the socket bottoms. They lift straight out; there are no clips or magnets. The rendered preview shows nine seated hearts and the tenth raised to illustrate the peg.

## Source and validation

`health_tracker.blend` contains the assembled preview and a collection with the heart and fit-test source parts. `build_tracker.py` is the editable generator; dimensions and clearance are constants at the top. Run it with Blender 5.2:

```text
blender --background --factory-startup --python build_tracker.py
```

STLs are checked for closed manifold geometry, positive volume, degenerate triangles, connected parts, and placement on the print bed. The standard seated heart is also checked for solid interference with the tray. Results are in `mesh_checks.json`; independent checks of the exported STL files are in `stl_checks.json`.

`assembled_preview.png` is rendered from the actual modeled parts. Files have been digitally checked, but have **not been physically test-printed**; use the fit strip before committing to the full tray.
