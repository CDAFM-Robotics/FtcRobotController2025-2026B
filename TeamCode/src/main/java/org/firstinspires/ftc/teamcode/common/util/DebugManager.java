package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DebugManager {

    // ─── Master switches ───────────────────────────────────────────
    public static boolean TELEMETRY_ENABLED = true;
    public static boolean ROBOT_LOG_ENABLED = true;

    // Optional: per-category switches
    public static boolean LOG_DRIVEBASE      = true;
    public static boolean LOG_PINPOINT   = true;
    public static boolean LOG_VISION     = true;
    public static boolean LOG_LAUNCHER   = true;
    public static boolean LOG_SPINDEXER  = true;
    public static boolean LOG_INTAKE     = true;
    public static boolean LOG_HUD        = true;
    public static boolean LOG_ROBOT        = true;

    private final Telemetry telemetry;
    private final String tag; // RobotLog tag, e.g. "DRIVEBASE" or "PINPOINT"

    public DebugManager(Telemetry telemetry, String tag) {
        this.telemetry = telemetry;
        this.tag = tag;
    }

    // ─── Telemetry ─────────────────────────────────────────────────

    public void addData(String caption, Object value) {
        if (TELEMETRY_ENABLED) {
            telemetry.addData(caption, value);
        }
    }

    public void addData(String caption, String format, Object... values) {
        if (TELEMETRY_ENABLED) {
            telemetry.addData(caption, String.format(format, values));
        }
    }

    public void addLine(String line) {
        if (TELEMETRY_ENABLED) {
            telemetry.addLine(line);
        }
    }

    public void update() {
        if (TELEMETRY_ENABLED) {
            telemetry.update();
        }
    }

    // ─── RobotLog — format string + args ──────────────────────────

    /** Info — RobotLog.ii */
    public void log(String format, Object... values) {
        if (ROBOT_LOG_ENABLED) {
            RobotLog.ii(tag, String.format(format, values));
        }
    }

    /** Warning — RobotLog.ww */
    public void logWarn(String format, Object... values) {
        if (ROBOT_LOG_ENABLED) {
            RobotLog.ww(tag, String.format(format, values));
        }
    }

    /** Error — always logs, ignores ROBOT_LOG_ENABLED master switch */
    public void logError(String format, Object... values) {
        RobotLog.ee(tag, String.format(format, values));
    }

    /** Verbose — RobotLog.vv */
    public void logVerbose(String format, Object... values) {
        if (ROBOT_LOG_ENABLED) {
            RobotLog.vv(tag, String.format(format, values));
        }
    }

    // ─── Category-aware helpers ────────────────────────────────────

    public void robot(String line) {
        if (TELEMETRY_ENABLED && LOG_ROBOT) {
            telemetry.addLine("[bot] " + line);
        }
    }

    public void robot(String caption, Object value) {
        if (TELEMETRY_ENABLED && LOG_ROBOT) {
            telemetry.addData("[bot] " + caption, value);
        }
    }

    public void robot(String caption, String format, Object... values) {
        if (TELEMETRY_ENABLED && LOG_ROBOT) {
            telemetry.addData("[bot] " + caption, String.format(format, values));
        }
    }

    public void pinpoint(String line) {
        if (TELEMETRY_ENABLED && LOG_PINPOINT) {
            telemetry.addLine("[PIN] " + line);
        }
    }

    public void pinpoint(String caption, Object value) {
        if (TELEMETRY_ENABLED && LOG_PINPOINT) {
            telemetry.addData("[PIN] " + caption, value);
        }
    }

    public void pinpoint(String caption, String format, Object... values) {
        if (TELEMETRY_ENABLED && LOG_PINPOINT) {
            telemetry.addData("[PIN] " + caption, String.format(format, values));
        }
    }

    public void drivebase(String line) {
        if (TELEMETRY_ENABLED && LOG_DRIVEBASE) {
            telemetry.addLine("[DIV] " + line);
        }
    }

    public void drivebase(String caption, Object value) {
        if (TELEMETRY_ENABLED && LOG_DRIVEBASE) {
            telemetry.addData("[DRV] " + caption, value);
        }
    }

    public void drivebase(String caption, String format, Object... values) {
        if (TELEMETRY_ENABLED && LOG_DRIVEBASE) {
            telemetry.addData("[DRV] " + caption, String.format(format, values));
        }
    }

    public void vision(String line) {
        if (TELEMETRY_ENABLED && LOG_VISION) {
            telemetry.addLine("[VIS] " + line);
        }
    }

    public void vision(String caption, Object value) {
        if (TELEMETRY_ENABLED && LOG_VISION) {
            telemetry.addData("[VIS] " + caption, value);
        }
    }

    public void vision(String caption, String format, Object... values) {
        if (TELEMETRY_ENABLED && LOG_VISION) {
            telemetry.addData("[VIS] " + caption, String.format(format, values));
        }
    }

    public void intake(String line) {
        if (TELEMETRY_ENABLED && LOG_INTAKE) {
            telemetry.addLine("[INT] " + line);
        }
    }

    public void intake(String caption, Object value) {
        if (TELEMETRY_ENABLED && LOG_INTAKE) {
            telemetry.addData("[INT] " + caption, value);
        }
    }

    public void intake(String caption, String format, Object... values) {
        if (TELEMETRY_ENABLED && LOG_INTAKE) {
            telemetry.addData("[INT] " + caption, String.format(format, values));
        }
    }

    public void launcher(String line) {
        if (TELEMETRY_ENABLED && LOG_LAUNCHER) {
            telemetry.addLine("[LAU] " + line);
        }
    }

    public void launcher(String caption, Object value) {
        if (TELEMETRY_ENABLED && LOG_LAUNCHER) {
            telemetry.addData("[LAU] " + caption, value);
        }
    }

    public void launcher(String caption, String format, Object... values) {
        if (TELEMETRY_ENABLED && LOG_LAUNCHER) {
            telemetry.addData("[LAU] " + caption, String.format(format, values));
        }
    }

    public void spindexer(String line) {
        if (TELEMETRY_ENABLED && LOG_SPINDEXER) {
            telemetry.addLine("[SPI] " + line);
        }
    }

    public void spindexer(String caption, Object value) {
        if (TELEMETRY_ENABLED && LOG_SPINDEXER) {
            telemetry.addData("[SPI] " + caption, value);
        }
    }

    public void spindexer(String caption, String format, Object... values) {
        if (TELEMETRY_ENABLED  && LOG_SPINDEXER) {
            telemetry.addData("[SPI] " + caption, String.format(format, values));
        }
    }

}
