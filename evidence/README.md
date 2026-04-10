# Final Report Evidence Bundle

This directory collects the non-code material referenced by the final report so the full submission can be reviewed through GitHub.

## Where To Find Things

- Main report figures: `Figures/main_figures/`
- Telemetry plots: `telemetry/plots/`
- Processed telemetry logs: `telemetry/runs/`
- Demonstration videos: `videos/`
- Firmware source code referenced in the report: `../M2_HW/` and `../libs/`

## Important Name Mapping

The report text refers to `FR4_proof.png` and `FR5_proof.png`.
The preserved image files in this repository are named:

- `telemetry/plots/rq4_proof.png`
- `telemetry/plots/rq5_proof.png`

## Video Scope

Only a curated subset of videos is included here so the repository remains practical to browse and clone on standard GitHub hosting.
The included videos are representative balancing and demonstration captures that stay within GitHub's regular per-file size limit.

## Code Map

The report's Appendix 1 and Appendix 2 references correspond to:

- Application layer: `../M2_HW/app_main_cap.c`, `../M2_HW/CONTROL.c`, `../M2_HW/CONTROL.h`, `../M2_HW/APP_STATE.h`, `../M2_HW/APP_CONFIG.h`, `../M2_HW/SETTINGS.c`, `../M2_HW/SETTINGS.h`, `../M2_HW/TELEMETRY.c`, `../M2_HW/TELEMETRY.h`
- Service, driver, HAL, and utility modules: `../libs/` and `../libs/SOURCE/`

## Notes

- The report document itself is intentionally not stored in this public repository.
- The telemetry figures in `telemetry/plots/` are the evidence artifacts cited in Appendix 3.
- The `telemetry/plots/source_pdfs/` directory preserves the source plot exports alongside the PNGs.
