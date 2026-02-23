package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.commands.Paths;
import org.firstinspires.ftc.teamcode.pedropathing.commands.SubsystemCommands;

@Autonomous(name = "Pedro's Cool Auto Program", group = "Testing")
public class BlueBackPedroPathingAuto extends OpMode {

    Follower follower;
    Robot robot;

    private enum State {
        INIT,
        READY,
        SHOOT,
        FAR_PICKUP,
        ZONE_PICKUP,
    }

    private State[] order;
    private State[] endLoop;

    private State state;

    int currentState = 0;

    private State getNextState() {
        if (currentState < order.length) {
            return order[currentState++];
        }
        else {
            return endLoop[(currentState++ - order.length) % endLoop.length];
        }
    }

    Paths paths;

    @Override
    public void init() {
        state = State.INIT;

        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, false);

        follower.setStartingPose(new Pose(56, 8.5, Math.PI));

        SubsystemCommands subsystemCommands = new SubsystemCommands(robot);
        paths = new Paths(follower);

        order = new State[] {State.SHOOT, State.FAR_PICKUP, State.SHOOT};
        endLoop = new State[] {State.ZONE_PICKUP, State.SHOOT};

        state = State.READY;
    }

    @Override
    public void init_loop() {
        robot.getLauncher().autoUpdateTurretPID(-64);
    }

    @Override
    public void start() {
        state = getNextState();
    }

    boolean newState = true;

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        telemetry.addData("State", state);
        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Drive State", driveState);

        switch (state) {
            case SHOOT:
                updateShoot();
                break;

            case FAR_PICKUP:
                updateDrive();
                break;

            default:
                //yup this is default
        }

        telemetry.update();
    }

    private enum ShootState {
        INIT,
        PREPARING,
        SHOOTING,
        FINISHED
    }

    ShootState shootState = ShootState.INIT;

    private void updateShoot() {
        if (newState) {
            shootState = ShootState.INIT;
        }

        switch (shootState) {
            case INIT:
                robot.getLauncher().setAutoVelocity(1565);
                robot.getLauncher().autoUpdateTurretPID(-64);

                if (robot.getLauncher().getLauncherVelocity() >= 1540 && Math.abs(robot.getLauncher().getTurretDegrees() + 64) < 5) {
                    shootState = ShootState.SHOOTING;
                }
                else {
                    shootState = ShootState.PREPARING;
                }
                break;
            case PREPARING:

                telemetry.addData("Launcher Velocity", robot.getLauncher().getLauncherVelocity());
                telemetry.addData("Turret Angle", robot.getLauncher().getTurretDegrees());

                robot.getLauncher().autoUpdateTurretPID(-64);
                if (robot.getLauncher().getLauncherVelocity() >= 1540 && Math.abs(robot.getLauncher().getTurretDegrees() + 64) < 5) {
                    shootState = ShootState.SHOOTING;
                }
                break;
            case SHOOTING:
                robot.shootAllBalls();

                if (robot.isNoArtifacts()) {
                    shootState = ShootState.FINISHED;
                }
                break;
            case FINISHED:
                telemetry.addData("Shoot State", shootState);
                state = getNextState();
                newState = true;
        }

        if (driveState != DriveState.FINISHED) {
            newState = false;
        }
    }

    private enum DriveState {
        INIT,
        PICKUP_1,
        PICKUP_2,
        PICKUP_0,
        PREP_FOR_SHOOT,
        FINISHED
    }

    DriveState driveState = DriveState.INIT;

    private void updateDrive() {
        if (newState) {
            driveState = DriveState.INIT;
        }

        switch (driveState) {
            case INIT:
                follower.followPath(paths.getBlueFarPickupThirdMark(), true);
                robot.getIntake().setIntakeMotorPower(1);
                robot.getIndexer().rotateToPosition(0);
                driveState = DriveState.PICKUP_0;
                break;
            case PICKUP_0:
                follower.update();
                if (!robot.getIndexer().getPredictedColor(
                        robot.getIndexer().colorSensorIntakeL.getNormalizedColors(),
                        robot.getIndexer().colorSensorIntakeR.getNormalizedColors(),
                        ((DistanceSensor) robot.getIndexer().colorSensorIntakeL).getDistance(DistanceUnit.CM),
                        ((DistanceSensor) robot.getIndexer().colorSensorIntakeR).getDistance(DistanceUnit.CM)).equals(ArtifactColor.NONE)) {
                    driveState = DriveState.PICKUP_1;
                    robot.getIndexer().rotateToPosition(1);
                }

                break;
            case PICKUP_1:
                follower.update();
                if (!robot.getIndexer().getPredictedColor(
                    robot.getIndexer().colorSensorIntakeL.getNormalizedColors(),
                    robot.getIndexer().colorSensorIntakeR.getNormalizedColors(),
                    ((DistanceSensor) robot.getIndexer().colorSensorIntakeL).getDistance(DistanceUnit.CM),
                    ((DistanceSensor) robot.getIndexer().colorSensorIntakeR).getDistance(DistanceUnit.CM)).equals(ArtifactColor.NONE)) {

                    driveState = DriveState.PICKUP_2;
                    robot.getIndexer().rotateToPosition(1);
                }
                break;
            case PICKUP_2:
                follower.update();
                if (!robot.getIndexer().getPredictedColor(
                    robot.getIndexer().colorSensorIntakeL.getNormalizedColors(),
                    robot.getIndexer().colorSensorIntakeR.getNormalizedColors(),
                    ((DistanceSensor) robot.getIndexer().colorSensorIntakeL).getDistance(DistanceUnit.CM),
                    ((DistanceSensor) robot.getIndexer().colorSensorIntakeR).getDistance(DistanceUnit.CM)).equals(ArtifactColor.NONE)) {

                    driveState = DriveState.PREP_FOR_SHOOT;
                    robot.getIndexer().rotateToPosition(0);
                    robot.getLauncher().setAutoVelocity(1565);
                }
                break;
            case PREP_FOR_SHOOT:
                follower.update();

                break;
            case FINISHED:
                state = getNextState();
                newState = true;
                break;
        }

        if (driveState != DriveState.FINISHED) {
            newState = false;
        }

        if (!follower.isBusy()) {
            driveState = DriveState.FINISHED;
        }
    }
}
