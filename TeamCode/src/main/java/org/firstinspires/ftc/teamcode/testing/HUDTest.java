package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.subsystems.Hud;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.common.util.QwiicLEDStick;

import java.util.Random;


@TeleOp (name="HUDTest", group = "testing")
public class HUDTest extends LinearOpMode {
  private Blinker control_Hub;
  public ElapsedTime elapsedTime = new ElapsedTime();
  static TelemetryManager telemetryM;

  private Hud hud;
  public ArtifactColor[] artifactColorArray = new ArtifactColor[]{ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE};


  // ColorTable2 is the possible COLORS for 2025-2026 Decode
  public enum ColorTable {PURPLE, GREEN, NONE, RED, WHITE}


  public ArtifactColor ball1 = ArtifactColor.NONE;
  public ArtifactColor ball2 = ArtifactColor.NONE;
  public ArtifactColor ball3 = ArtifactColor.NONE;


  public ElapsedTime timeSinceLastHUDChange = new ElapsedTime();
  public ElapsedTime safetyTimer = new ElapsedTime();


  @Override
  public void runOpMode() {

    // TODO Panels telemetry and robot drawing
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    hud = new Hud(hardwareMap, telemetry);
    timeSinceLastHUDChange.reset();
    safetyTimer.reset();

    waitForStart();
    if (isStopRequested()) {
      return;
    }
    resetRuntime();

    while (opModeIsActive()) {
      elapsedTime.reset();
      // Lets generate some random "balls" at 2hz refresh rate
      if (timeSinceLastHUDChange.milliseconds() >= 500) {
        ball1 = randomBall();
        ball2 = randomBall();
        ball3 = randomBall();
        timeSinceLastHUDChange.reset();
      }
      hud.setBalls(ball1, ball2, ball3);
      // hud.UpdateBallUI2();
      // hud.UpdateBallUI();
      if (safetyTimer.milliseconds() > 25)
      {
        hud.update();
        safetyTimer.reset();
      }
      // RobotLog.d("looptime: %.2f", elapsedTime.milliseconds());
      telemetryM.addData("loopTime", elapsedTime.milliseconds());
      telemetryM.update();
    }
  }


  // (optional) Testing Helper generates random pixel colors
  public static ArtifactColor randomBall()  {
    Random random = new Random();
    return ArtifactColor.values()[random.nextInt(4)];
  }
}
