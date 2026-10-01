# Limelight 3A pipeline backup

Exported 2026-10-01 from the 2025-2026 robot's Limelight 3A (firmware 2026.0) over USB at
`172.29.0.1`.

## What is here

| Slot | File | Type | Notes |
| --- | --- | --- | --- |
| 0 | `pipelines/0_PurpleBallDetector.vpr` | Color | Hue 130–179, exposure 2746, gain 28.3. `Robot.LLPipelines.PURPLE`. |
| 1 | `pipelines/1_GreenBallDetector.vpr` | Color | Hue 39–77, sat min 72, val min 54, exposure 3300, gain 26.3. The code calls this slot `YELLOW`. |
| 2 | `pipelines/2_Pipeline_Name.vpr` | Fiducial | Unused placeholder. The code calls this slot `BLUE`. |
| 3 | `pipelines/3_APRIL_TAG.vpr` | Fiducial | Full 3D, exposure 1073, gain 7.7, camera roll -7.6°. |
| 4 | `pipelines/4_Pipeline_Name.vpr` | Fiducial | Unused placeholder. **The code expects `MOTIF` here** (see below). |
| 5 | `pipelines/5_RED_GOAL.vpr` | Fiducial | AprilTag ID 24 only. |
| 6 | `pipelines/6_BLUE_GOAL.vpr` | Fiducial | AprilTag ID 20 only. |
| 7 | `pipelines/7_OBELISK.vpr` | Fiducial | AprilTag IDs 21, 22, 23. |
| 8–9 | `pipelines/8_Pipeline_Name.vpr`, `9_Pipeline_Name.vpr` | Fiducial | Unused placeholders. |

Slot numbers match the `Robot.LLPipelines` enum in `TeamCode`, which switches pipelines by ordinal.

- `fieldmap.fmap` is the field map loaded on the camera (FTC, AprilTags 20 and 24 only). Every slot
  reported the same map.
- `python/stock_template.py` is the Python script stored in every slot. It is Limelight's unmodified
  starter template, and no slot is a Python pipeline, so the camera does not run it.

The `.vpr` files are byte-for-byte what the camera returned from
`GET http://172.29.0.1:5807/pipeline-atindex?index=N`, which is the same content the web UI's
pipeline download button produces. They are left as single-line JSON so they stay exact.

The four placeholder files (slots 2, 4, 8, 9) are identical. During the export, loading slot 2 made
the firmware rewrite it with ten newer settings keys added at their default values; no existing
value changed. The file here is the slot as it was before that.

## What is NOT here

- **No MOTIF pipeline.** `Robot.java` lists slot 4 as `MOTIF`, in use, but slot 4 on the camera is a
  blank placeholder. The pipeline was not on the camera at export time.
- No neural network models: none of the slots is a neural pipeline.
- No snapshots (the camera reported none) and no custom calibration (the camera has only the factory
  one).

## Restoring

In the web UI (`http://172.29.0.1:5801` over USB), select the slot, then use the upload button next
to the pipeline name and pick the `.vpr` file. The field map (`.fmap`) and Python script have their
own upload controls in the pipeline settings.

## Re-exporting

```sh
for i in 0 1 2 3 4 5 6 7 8 9; do
  curl -s "http://172.29.0.1:5807/pipeline-atindex?index=$i" -o "slot$i.vpr"
done
```

The field map and Python scripts are only readable for the active slot. They were read over the
web UI's websocket (port 5805, `request_fieldmap` and `request_current_python_script_for_download`)
after switching to each slot in turn.
