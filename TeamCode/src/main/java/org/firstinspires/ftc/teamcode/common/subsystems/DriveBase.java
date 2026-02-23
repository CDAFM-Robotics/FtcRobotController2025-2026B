package org.firstinspires.ftc.teamcode.common.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;

public class DriveBase {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private Servo leftKickStand = null;
    private Servo rightKickStand = null;
    private Servo kickStandLight = null;

    // private IMU imu;

    GoBildaPinpointDriver pinpoint;
    private Pose2D pos;
    private boolean isRedSide;


    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry, boolean isRed) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        isRedSide = isRed;

        initializeDriveBaseDevices(isRed);
    }

    public void initializeDriveBaseDevices(boolean isRed) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        //rightKickStand = hardwareMap.get(Servo.class, "rightKickStand");
        //leftKickStand = hardwareMap.get(Servo.class, "leftKickStand");
        //kickStandLight = hardwareMap.get(Servo.class, "kickStandLight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initialize the kick stand servos
        //rightKickStand.setPosition(0.5);
        //leftKickStand.setPosition(0.5);

        // ground lights OFF
        //kickStandLight.setPosition(0.0);

        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();
        pinpoint.update();
        
        //read the pose value from autonomous or initialized it at start up location
        Pose2D startPose2D;
        // if the auto completed, use the value from end of auto
        if (RobotStaticValuesClass.autoCompleted) {
            startPose2D = RobotStaticValuesClass.savedPose;
        } else {
            startPose2D = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
        }

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(startPose2D);
        pinpoint.update();
        RobotLog.d("Pos x: %.2f, y: %2f, heading: %.2f", startPose2D.getX(DistanceUnit.INCH), startPose2D.getY(DistanceUnit.INCH), startPose2D.getHeading(AngleUnit.RADIANS));

    }

    public void resetIMU() {
        resetPinpointIMU();
    }

    public void resetPinpointIMU() {
        pinpoint.resetPosAndIMU();
    }

    public void configurePinpoint() {

        // TODO TESTING Orientation may be wrong.
        pinpoint.setOffsets(8, -3.25, DistanceUnit.INCH); //Tuned for 2026 Bot2 17Feb26 OK

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
        sleep(300);
    }

    public void setMotorPowers(double x, double y, double rx, double speed, boolean fieldCentric) {
//        RobotLog.d("pinpoint heading1 %.2f", pinpoint.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
//        telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));
        pinpoint.update();
        pos = pinpoint.getPosition();

        double heading = 0;
        double driverOffset;
        double adjustedHeading;
        if (fieldCentric) {
            heading = pos.getHeading(AngleUnit.RADIANS);
            // Apply 90-degree driver offset (in radians)
            if (isRedSide) {
                driverOffset = Math.toRadians(-90); // Change to -90 if opposite direction needed
            } else {
                driverOffset = Math.toRadians(90);
            }

            adjustedHeading = -(heading - driverOffset);
            //if (adjustedHeading > 180)  adjustedHeading -= 360;
            //if (adjustedHeading < -180) adjustedHeading += 360;
        } else {
            adjustedHeading = 0;
        }

        double rotX = x * Math.cos(adjustedHeading) - y * Math.sin(adjustedHeading);
        double rotY = x * Math.sin(adjustedHeading) + y * Math.cos(adjustedHeading);

        // put strafing factors here
        rotX = rotX * 1;
        rotY = rotY * 1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = ((rotY + rotX) * speed + rx) / denominator;
        double backLeftPower = ((rotY - rotX) * speed + rx) / denominator;
        double frontRightPower = ((rotY - rotX) * speed - rx) / denominator;
        double backRightPower = ((rotY + rotX) * speed - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        telemetry.addData("Pinpoint", "Heading %.2f, Pos %.2f", heading, pos.getX(DistanceUnit.MM));
        telemetry.addData("fieldCentric", fieldCentric);
        telemetry.addData("powers", "front left: %.2f, front right: %.2f, back left: %.2f, back right: %.2f", frontLeftPower * speed * 100, frontRightPower * speed * 100, backLeftPower * speed * 100, backRightPower * speed * 100);
    }

    public void setIndividualMotorPowers(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        backLeftMotor.setPower(backLeftPower);
    }

/*    public void setKickStand() {
        rightKickStand.setPosition(0.0);
        leftKickStand.setPosition(1.0);
    }

    public void resetKickStand() {
        rightKickStand.setPosition(0.5);
        leftKickStand.setPosition(0.5);
    }

    public void setKickStandLight() {
        kickStandLight.setPosition(1.0);
    }

    public void resetKickStandLight(){
        kickStandLight.setPosition(0.0);
    }

    public void adjustKickStandLight(double power){
        kickStandLight.setPosition(power);
    }*/

    public double getPinPointPosX() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getPinPointPosY() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getPinPointHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

}