package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.util.QwiicLEDStick;
import androidx.annotation.ColorInt;

import java.util.Random;


@TeleOp (group = "testing", name = "LEDStickTest")
public class LEDStickTest extends LinearOpMode {
  private Blinker control_Hub;
  public ElapsedTime elapsedTime = new ElapsedTime();
  public boolean WaitingForHUD = false;
  public boolean WaitForBulb = false;
  public boolean[] lights = new boolean[10];
  public boolean[] lightsFront=new boolean[10];

  // Define the LEDStrip
  public QwiicLEDStick ledstrip;
  public QwiicLEDStick ledstripFront;

  public static int LED_STICK_BRIGHTNESS=5; // Brightness (1-31)
  public static int LED_STICK_TOTAL_LEDS=10; // How many Total LED there are to control
  public static int ms_delay=10;

  // ColorTable2 is the possible COLORS for 2025-2026 Decode
  public enum ColorTable {PURPLE, GREEN, NONE, RED, WHITE}

  // instance vars to hold Ball colors (2025-2026 Decode)
  public ColorTable ball1= ColorTable.NONE, last1 = ColorTable.NONE;
  public ColorTable ball2= ColorTable.NONE, last2 = ColorTable.NONE;
  public ColorTable ball3= ColorTable.NONE, last3 = ColorTable.NONE;
  public ColorTable aimLED = ColorTable.NONE, lastAimLED = ColorTable.NONE;

  public ElapsedTime timeSinceLastHUDChange = new ElapsedTime();

  // Pre-defined color values for valid Balls (2025-2026) (same order as Enum)
  public static @ColorInt int[] Balls = new int[]{
          //Color.parseColor("purple"), // purple
          Color.rgb(255,0,255), // purple
          Color.rgb(0,255,0), // green
          Color.rgb(0, 0, 0), // off
          Color.rgb(255,0,0), // red
          Color.parseColor("silver"), // white
  };

  public static @ColorInt int[] colors_all = new int[]{
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
          Color.rgb(0, 0, 0),
  };

  @Override
  public void runOpMode() {


    // initialize the SparkFun Qwiic LED Strip Apa102C
    // remember to add the i2c port in robot config (port #1 = i2c bus 1, etc)
    ledstrip = hardwareMap.get(QwiicLEDStick.class, "ledstrip");
    ledstrip.changeLength(LED_STICK_TOTAL_LEDS); // limit addressable LED to number of LED installed
    ledstrip.setBrightness(LED_STICK_BRIGHTNESS);// 10 LEDs at brightness 31 generates 660ma current
    ledstripFront = hardwareMap.get(QwiicLEDStick.class, "ledstripFront");
    ledstripFront.changeLength(LED_STICK_TOTAL_LEDS);
    ledstripFront.setBrightness(LED_STICK_BRIGHTNESS);
    timeSinceLastHUDChange.reset();

    // Set LAST to an unused and different color to ensure it gets updated once at init to clear carry-over.
    last1 = ColorTable.WHITE;
    last2 = ColorTable.WHITE;
    last3 = ColorTable.WHITE;
    lastAimLED = ColorTable.WHITE;
    // RobotLog.d("HUD: Init %s %s %s %s - T:  %.2f ms", ball1,ball2,ball3,aimLED,timeSinceLastHUDChange.milliseconds());

    waitForStart();
    if (isStopRequested()) {
      return;
    }
    resetRuntime();

    while (opModeIsActive()) {
      // Lets generate some random "balls" at 2hz refresh rate
      if (elapsedTime.milliseconds() >= 500) {
        ball1 = randomBall();
        ball2 = randomBall();
        ball3 = randomBall();
        elapsedTime.reset();
        // resetLights();
        UpdateBallUI();
      }
      // UpdateBalls();
      // UpdateBallUI2();

      RobotLog.d("Hi");


    }
  }



  public void UpdateBalls() {

    // Sleep for 5ms between command to avoid overwhelming i2c device with
    // messages (0 causes glitches & strip freezes)

    // //sleep for 5ms between command to avoid overwhelming i2c device with
    // messages (0 causes glitches & strip freezes)



    if (ball1 != last1) { // only send update if different

      // Ball1 (bottom) LEDs 0-2
      if (timeSinceLastHUDChange.milliseconds() > ms_delay && !lights[0]) {
        ledstrip.setColor(0, Balls[ball1.ordinal()]);
        lights[0] = true;
        timeSinceLastHUDChange.reset();

      }
      //sleep(ms_delay);

      if (timeSinceLastHUDChange.milliseconds() > ms_delay && !lightsFront[0]) {
        ledstripFront.setColor(0, Balls[ball1.ordinal()]);
        lightsFront[0] = true;
        timeSinceLastHUDChange.reset();

      }
      //sleep(ms_delay);

      if (timeSinceLastHUDChange.milliseconds() > ms_delay && !lights[1]) {
        ledstrip.setColor(1, Balls[ball1.ordinal()]);
        lights[1] = true;
        timeSinceLastHUDChange.reset();

      }
      //sleep(ms_delay);

      if (timeSinceLastHUDChange.milliseconds() > ms_delay && !lightsFront[1]) {
        ledstripFront.setColor(1, Balls[ball1.ordinal()]);
        lightsFront[1]=true;
        timeSinceLastHUDChange.reset();

      }
      //sleep(ms_delay);

      if (timeSinceLastHUDChange.milliseconds() > ms_delay && !lights[2]) {
        ledstrip.setColor(2, Balls[ball1.ordinal()]);
        lights[2]=true;
        timeSinceLastHUDChange.reset();

      }
      //sleep(ms_delay);

      if (timeSinceLastHUDChange.milliseconds() > ms_delay && !lightsFront[2]) {
        ledstripFront.setColor(2, Balls[ball1.ordinal()]);
        lightsFront[2]=true;
        timeSinceLastHUDChange.reset();

        last1 = ball1;  // Set this BALL SAME so it won't update

      }
      //sleep(ms_delay);
    }

    if (ball2 != last2) {
      // Ball2

      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstrip.setColor(3, Balls[ball2.ordinal()]);
        timeSinceLastHUDChange.reset();
      }
      //sleep(ms_delay);
      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstripFront.setColor(3, Balls[ball2.ordinal()]);
        timeSinceLastHUDChange.reset();
      }
      //sleep(ms_delay);

      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstrip.setColor(4, Balls[ball2.ordinal()]);
        timeSinceLastHUDChange.reset();
      }
      //sleep(ms_delay);

      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstripFront.setColor(4, Balls[ball2.ordinal()]);
        timeSinceLastHUDChange.reset();
      }
      //sleep(ms_delay);
      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstrip.setColor(5, Balls[ball2.ordinal()]);
        timeSinceLastHUDChange.reset();
      }
      //sleep(ms_delay);

      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstripFront.setColor(5, Balls[ball2.ordinal()]);
        timeSinceLastHUDChange.reset();

        last2 = ball2;
      }
      //sleep(ms_delay);

    }
    if (ball3 != last3) {

      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstrip.setColor(6, Balls[ball3.ordinal()]);
        timeSinceLastHUDChange.reset();
      }

      //sleep(ms_delay);
      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstripFront.setColor(6, Balls[ball3.ordinal()]);
        timeSinceLastHUDChange.reset();
      }

      //sleep(ms_delay);
      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstrip.setColor(7, Balls[ball3.ordinal()]);
        timeSinceLastHUDChange.reset();
      }
      //sleep(ms_delay);
      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstripFront.setColor(7, Balls[ball3.ordinal()]);
        timeSinceLastHUDChange.reset();
      }

      //sleep(ms_delay);
      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstrip.setColor(8, Balls[ball3.ordinal()]);
        timeSinceLastHUDChange.reset();
      }
      //sleep(ms_delay);
      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstripFront.setColor(8, Balls[ball3.ordinal()]);
        timeSinceLastHUDChange.reset();

        // Ball3
        last3 = ball3;
      }
      //sleep(ms_delay);
    }


    // set aiming led
    if (aimLED != lastAimLED) {

      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstrip.setColor(9, Balls[aimLED.ordinal()]);
        timeSinceLastHUDChange.reset();
      }
      //sleep(ms_delay);
      if (timeSinceLastHUDChange.milliseconds() > ms_delay) {
        ledstripFront.setColor(9, Balls[aimLED.ordinal()]);
        timeSinceLastHUDChange.reset();

        lastAimLED = aimLED;
      }
      //sleep(ms_delay);
    }
  }

  public void resetLights()
  {
    for (int i=0;i<10;i++){
      lights[i] = false;
      lightsFront[i] = false;
    }
  }


  public void UpdateBallUI2() {

    if (( (ball1 != last1) || (ball2 != last2) || (ball3 != last3) || (aimLED != lastAimLED))) {
      // RobotLog.d("HUD: %s %s %s %s - T:  %.2f ms", ball1,ball2,ball3,aimLED,timeSinceLastHUDChange.milliseconds());
      if (ball1 != last1) {
        last1 = ball1;
        colors_all[0] = Balls[ball1.ordinal()];
        colors_all[1] = Balls[ball1.ordinal()];
        colors_all[2] = Balls[ball1.ordinal()];
      }
      if (ball2 != last2) {
        last2 = ball2;
        colors_all[3] = Balls[ball2.ordinal()];
        colors_all[4] = Balls[ball2.ordinal()];
        colors_all[5] = Balls[ball2.ordinal()];
      }
      if (ball3 != last3) {
        last3 = ball3;
        colors_all[6] = Balls[ball3.ordinal()];
        colors_all[7] = Balls[ball3.ordinal()];
        colors_all[8] = Balls[ball3.ordinal()];
      }
      if (aimLED != lastAimLED) {
        lastAimLED = aimLED;
        colors_all[9] = Balls[aimLED.ordinal()];
      }

      // RobotLog.d("HUD: colors_all [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
//                    colors_all[0],colors_all[1],colors_all[2],
//                    colors_all[3],colors_all[4],colors_all[5],
//                    colors_all[6],colors_all[7],colors_all[8],
//                    colors_all[9]);

      if (timeSinceLastHUDChange.milliseconds() > 500 && !lights[0]) {
        ledstrip.setColors(colors_all);
        lights[0] = true;
        timeSinceLastHUDChange.reset();
      }
      if (timeSinceLastHUDChange.milliseconds() > 500 && !lights[1]) {
        ledstripFront.setColors(colors_all);
        lights[1] = true;
        timeSinceLastHUDChange.reset();
      }

    }
  }


  public void UpdateBallUI() {


    // //sleep for 5ms between command to avoid overwhelming i2c device with
    // messages (0 causes glitches & strip freezes)

    RobotLog.d("Update Start");
    if (ball1 != last1 ) { // only send update if different

      last1 = ball1;
      // Ball1 (bottom) LEDs 0-2
      ledstrip.setColor(0, Balls[ball1.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(0, Balls[ball1.ordinal()]);
      sleep(ms_delay);
      ledstrip.setColor(1, Balls[ball1.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(1, Balls[ball1.ordinal()]);
      sleep(ms_delay);
      ledstrip.setColor(2, Balls[ball1.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(2, Balls[ball1.ordinal()]);
      sleep(ms_delay);
    }

    if (ball2 != last2) {
      last2 = ball2;
      // Ball2
      ledstrip.setColor(3, Balls[ball2.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(3, Balls[ball2.ordinal()]);
      sleep(ms_delay);
      ledstrip.setColor(4, Balls[ball2.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(4, Balls[ball2.ordinal()]);
      sleep(ms_delay);
      ledstrip.setColor(5, Balls[ball2.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(5, Balls[ball2.ordinal()]);
      sleep(ms_delay);

    }
    if (ball3 != last3) {
      // Drop2_sensor (bottom) LEDs 5-8
      last3 = ball3;
      ledstrip.setColor(6, Balls[ball3.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(6, Balls[ball3.ordinal()]);

      sleep(ms_delay);
      ledstrip.setColor(7, Balls[ball3.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(7, Balls[ball3.ordinal()]);

      sleep(ms_delay);
      ledstrip.setColor(8, Balls[ball3.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(8, Balls[ball3.ordinal()]);
      sleep(ms_delay);
    }
    // set aiming led
    if (aimLED != lastAimLED) {
      lastAimLED = aimLED;
      ledstrip.setColor(9, Balls[aimLED.ordinal()]);
      sleep(ms_delay);
      ledstripFront.setColor(9, Balls[aimLED.ordinal()]);
      sleep(ms_delay);
    }
    RobotLog.d("UpdateUI Done");

  }


  // (optional) Testing Helper generates random pixel colors
  public static ColorTable randomBall()  {
    Random random = new Random();
    return ColorTable.values()[random.nextInt(2)];
  }
}
