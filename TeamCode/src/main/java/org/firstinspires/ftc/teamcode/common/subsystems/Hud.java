package org.firstinspires.ftc.teamcode.common.subsystems;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.common.util.QwiicLEDStick;

// TODO notes for future
// underlying i2cSyncSimple doesn't seem to be honoring I2CWaitControl.WRITTEN
// Device driver assumes it does, and issues successive i2c write calls, which then overwhelm device
// This breaks setting all_colors by array vs setting a single = individual call
// TODO Experiment with writeCoalescing and/or write a method to construct full i2cpacket by hand
// TODO to get rid of the internal wait times between calls (~18ms)


public class Hud {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public QwiicLEDStick ledstripRear;
    public QwiicLEDStick ledstripFront;

    public static int LED_STICK_BRIGHTNESS=5; // Brightness (1-31)
    public static int LED_STICK_TOTAL_LEDS=10; // How many Total LED there are to control
    public static int ms_delay=8; // 7 is too short, and get some rainbow colors;

    public ElapsedTime safety = new ElapsedTime(); // timer to prevent flooding i2c

    // ColorTable2 is the possible COLORS for 2025-2026 Decode
    public enum ColorTable {PURPLE, GREEN, NONE, RED, WHITE}

    public enum HudUpdateState {
        INIT,
        BALL1,
        BALL2,
        BALL3,
        READY_TO_UPDATE
    }

    HudUpdateState hudState = HudUpdateState.INIT;

    // instance vars to hold Ball colors (2025-2026 Decode)
    public ColorTable ball1= ColorTable.NONE, last1 = ColorTable.NONE;
    public ColorTable ball2= ColorTable.NONE, last2 = ColorTable.NONE;
    public ColorTable ball3= ColorTable.NONE, last3 = ColorTable.NONE;
    public ColorTable aimLED = ColorTable.NONE, lastAimLED = ColorTable.NONE;

    // Pre-defined color values for valid Balls (2025-2026) (same order as Enum)
    public static @ColorInt int[] Balls = new int[]{
            Color.rgb(255,0,255), // purple (direct)
            Color.rgb(0,255,0), // green
            Color.rgb(0,0,0), // off
            Color.parseColor("red"), // red
            Color.parseColor("silver") // white
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

    public Hud(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initializeHUD();
    }

    public void initializeHUD() {
        safety.reset();
        ledstripRear = hardwareMap.get(QwiicLEDStick.class, "ledstrip");
        // TODO 10 LED is already the default for QWIIKLED Stick. no need to waste the time here
//        ledstripRear.changeLength(LED_STICK_TOTAL_LEDS); // limit addressable LED to number of LED installed
//        while (safety.milliseconds()<ms_delay){}
//        safety.reset();
        ledstripRear.setBrightness(LED_STICK_BRIGHTNESS);// 10 LEDs at brightness 31 generates 660ma current
        while (safety.milliseconds()<ms_delay){}
        safety.reset();


        // Set LAST to an unused and different color to ensure it gets updated once at init to clear carry-over.
        last1 = ColorTable.WHITE;
        last2 = ColorTable.WHITE;
        last3 = ColorTable.WHITE;
        lastAimLED = ColorTable.WHITE;
    }

    // TODO this one doesn't work due to driver issue (multiple i2c write without wait)
    public void UpdateBallUI2() {

        if (((ball1 != last1) || (ball2 != last2) || (ball3 != last3) || (aimLED != lastAimLED)) && safety.milliseconds() >= 250 )
        {
            if (ball1 != last1)
            {
                last1 = ball1;
                colors_all[0] = Balls[ball1.ordinal()];
                colors_all[1] = Balls[ball1.ordinal()];
                colors_all[2] = Balls[ball1.ordinal()];
            }
            if (ball2 != last2)
            {
                last2 = ball2;
                colors_all[3] = Balls[ball2.ordinal()];
                colors_all[4] = Balls[ball2.ordinal()];
                colors_all[5] = Balls[ball2.ordinal()];
            }
            if (ball3 != last3)
            {
                last3 = ball3;
                colors_all[6] = Balls[ball3.ordinal()];
                colors_all[7] = Balls[ball3.ordinal()];
                colors_all[8] = Balls[ball3.ordinal()];
            }
            if (aimLED != lastAimLED)
            {
                colors_all[9] = Balls[2]; // 2 = off
            }


            ledstripRear.setColors(colors_all);
            safety.reset();
        }

    }

    // TODO Hud.update() will run a continuous state machine that only updates one ball per update
    // TODO spreading the long running update task across multiple control loops also need to
    // TODO throttle calls in the main program by ~25ms per call to update
    public void update()
    {
        safety.reset();
        switch (hudState) {
            case INIT:
                hudState = HudUpdateState.BALL1;
            case BALL1:
                if (ball1 != last1) {
                    last1 = ball1;
                    ledstripRear.setColor(0, Balls[ball1.ordinal()]);
                    while (safety.milliseconds()<ms_delay){}
                    safety.reset();
                    ledstripRear.setColor(1, Balls[ball1.ordinal()]);
                    while (safety.milliseconds()<ms_delay){}
                    safety.reset();
                    ledstripRear.setColor(2, Balls[ball1.ordinal()]);
                    hudState = HudUpdateState.BALL2;
                    return;
                }
                hudState = HudUpdateState.BALL2;
                return;
            case BALL2:
                if (ball2 != last2) {
                    last2 = ball2;
                    ledstripRear.setColor(3, Balls[ball2.ordinal()]);
                    while (safety.milliseconds()<ms_delay){}
                    safety.reset();
                    ledstripRear.setColor(4, Balls[ball2.ordinal()]);
                    while (safety.milliseconds()<ms_delay){}
                    safety.reset();
                    ledstripRear.setColor(5, Balls[ball2.ordinal()]);
                    hudState = HudUpdateState.BALL3;
                    return;
                }
                hudState = HudUpdateState.BALL3;
                return;
            case BALL3:
                if (ball3 != last3) {
                    last3 = ball3;
                    ledstripRear.setColor(6, Balls[ball3.ordinal()]);
                    while (safety.milliseconds()<ms_delay){}
                    safety.reset();
                    ledstripRear.setColor(7, Balls[ball3.ordinal()]);
                    while (safety.milliseconds()<ms_delay){}
                    safety.reset();
                    ledstripRear.setColor(8, Balls[ball3.ordinal()]);

                    hudState = HudUpdateState.BALL1;
                    return;
                }
                hudState = HudUpdateState.BALL1;
        }
    }

    // This is the older UpdateUI routine. kept here to show how to address individual
    // LED colors and the associated delay between I2C messages to prevent confusing the LED stick
    public void UpdateBallUI() {
        // //sleep for 5ms-7ms between command to avoid overwhelming i2c device with
        // messages (0 causes glitches & strip freezes)

        if (ball1 != last1 ) { // only send update if different
            last1 = ball1;
            // Ball1 (bottom) LEDs 0-2
            ledstripRear.setColor(0, Balls[ball1.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(0, Balls[ball1.ordinal()]);
//            sleep(ms_delay);
            ledstripRear.setColor(1, Balls[ball1.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(1, Balls[ball1.ordinal()]);
//            sleep(ms_delay);
            ledstripRear.setColor(2, Balls[ball1.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(2, Balls[ball1.ordinal()]);
//            sleep(ms_delay);
        }

        if (ball2 != last2) {
            last2 = ball2;
            // Ball2
            ledstripRear.setColor(3, Balls[ball2.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(3, Balls[ball2.ordinal()]);
//            sleep(ms_delay);
            ledstripRear.setColor(4, Balls[ball2.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(4, Balls[ball2.ordinal()]);
//            sleep(ms_delay);
            ledstripRear.setColor(5, Balls[ball2.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(5, Balls[ball2.ordinal()]);
//            sleep(ms_delay);

        }
        if (ball3 != last3) {
            // Drop2_sensor (bottom) LEDs 5-8
            last3 = ball3;
            ledstripRear.setColor(6, Balls[ball3.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(6, Balls[ball3.ordinal()]);
//            sleep(ms_delay);
            ledstripRear.setColor(7, Balls[ball3.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(7, Balls[ball3.ordinal()]);
//            sleep(ms_delay);
            ledstripRear.setColor(8, Balls[ball3.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(8, Balls[ball3.ordinal()]);
//            sleep(ms_delay);
        }
        // set aiming led
        if (aimLED != lastAimLED) {
            lastAimLED = aimLED;
            ledstripRear.setColor(9, Balls[aimLED.ordinal()]);
            sleep(ms_delay);
//            ledstripFront.setColor(9, Balls[aimLED.ordinal()]);
//            sleep(ms_delay);
        }

    }

    public void setBalls(ArtifactColor b1, ArtifactColor b2, ArtifactColor b3)
    {
        ball1 = Hud.ColorTable.values()[b1.ordinal()];
        ball2 = Hud.ColorTable.values()[b2.ordinal()];
        ball3 = Hud.ColorTable.values()[b3.ordinal()];
    }

    public void setAimIndicator(Boolean aimOn)
    {
        if (aimOn) {
           aimLED = Hud.ColorTable.values()[Hud.ColorTable.RED.ordinal()];
        }
        else {
            aimLED = Hud.ColorTable.values()[Hud.ColorTable.NONE.ordinal()];
        }
    }

    public void AllOff()
    {
        ball1= Hud.ColorTable.NONE;
        ball2= Hud.ColorTable.NONE;
        ball3= Hud.ColorTable.NONE;
        aimLED= Hud.ColorTable.NONE;
    }

    public void sleep(int x)
    {
        try{
            Thread.sleep(x);
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException(e);
        }
    }

}
