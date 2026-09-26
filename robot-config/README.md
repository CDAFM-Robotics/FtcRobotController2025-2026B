# Robot Controller configuration

`Bot2withlimelight.xml` is the Control Hub hardware configuration for Bot2
(Control Hub + Expansion Hub 2 + Limelight 3A), as used during the April 2026
competition runs. It was recovered from the hub's backup
`Bot2withlimelightBk25Mar26.xml` after the active config was overwritten
on 2026-07-23, and cross-checked against the April 11 match logs and the
device names used in `TeamCode`.

## Restore to the hub

With the robot connected over adb:

```
adb push robot-config/Bot2withlimelight.xml /sdcard/FIRST/Bot2withlimelight.xml
```

Then pick `Bot2withlimelight` from the Driver Station configuration menu.
Changing the file name changes the configuration name shown on the Driver Station.

## Back up from the hub

```
adb pull /sdcard/FIRST/Bot2withlimelight.xml robot-config/Bot2withlimelight.xml
```

Commit after any change made through the Driver Station config editor.
