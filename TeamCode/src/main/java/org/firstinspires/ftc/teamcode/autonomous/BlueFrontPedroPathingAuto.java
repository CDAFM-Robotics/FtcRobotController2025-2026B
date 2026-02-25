package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.commands.Paths;
import org.firstinspires.ftc.teamcode.pedropathing.commands.SubsystemCommands;

import java.util.HashMap;
import java.util.LinkedList;

@Autonomous(name = "Blue Front Pedro Pathing", group = "Testing")
public class BlueFrontPedroPathingAuto extends OpMode {

    Follower follower;
    Robot robot;

    private enum State {
        INIT,
        READY,
        GO_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        MID_PICKUP_GATE,
        SHOOT,
        CLOSE_PICKUP,
        FAR_PICKUP
    }

    private LinkedList<State> order = new LinkedList<>();
    private LinkedList<State> endLoop = new LinkedList<>();

    private State state;

    int currentState = 0;

    private State getNextState() {
        if (currentState < order.size()) {
            return order.get(currentState++);
        }
        else {
            return endLoop.get((currentState++ - order.size()) % endLoop.size());
        }
    }

    Paths paths;

    @Override
    public void init() {
        state = State.INIT;

        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, false);

        follower.setStartingPose(new Pose(18.194, 121.659, Math.toRadians(143.5 - 180)));

        SubsystemCommands subsystemCommands = new SubsystemCommands(robot);
        paths = new Paths(follower);


        rows.put(0, true);
        rows.put(1, true);
        rows.put(2, true);

        labels.put(0, "Middle Mark + Gate");
        labels.put(1, "Close Mark");
        labels.put(2, "Far Mark");
    }

    private int maxRows = 3;
    private int currentRow = 0;
    private HashMap<Integer, Boolean> rows = new HashMap<>(4);
    private HashMap<Integer, String> labels = new HashMap<>(4);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void init_loop() {

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        robot.getLauncher().autoUpdateTurretPID(-45);

        if (state == State.INIT) {
            for (int i = 0; i < maxRows; i++) {
                if (i == currentRow) {
                    telemetry.addData("> " + labels.get(i), rows.get(i));
                }
                else {
                    telemetry.addData("   " + labels.get(i), rows.get(i));
                }
            }

            if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
                || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
                currentRow--;
            }

            if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
                || (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)) {
                currentRow++;
            }
            if (currentRow >= maxRows) {
                currentRow = maxRows - 1;
            }
            else if (currentRow < 0) {
                currentRow = 0;
            }

            if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left)
                || (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)
                || (currentGamepad1.dpad_right && !previousGamepad1.dpad_right)
                || (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)) {
                rows.put(currentRow, rows.get(currentRow));
            }

            if (currentGamepad1.a || currentGamepad2.a) {
                order.add(State.GO_TO_SHOOT_POS);
                order.add(State.SHOOT_PRELOAD);
                if (rows.get(0)) {
                    order.add(State.MID_PICKUP_GATE);
                    order.add(State.SHOOT);
                }
                if (rows.get(1)) {
                    order.add(State.CLOSE_PICKUP);
                    order.add(State.SHOOT);
                }
                if (rows.get(2)) {
                    order.add(State.FAR_PICKUP);
                    order.add(State.SHOOT);
                }
                state = State.READY;
            }
        }
    }

    @Override
    public void start() {
        if (state != State.READY) {
            throw new RuntimeException("Program was not locked in before running");
        }


        state = getNextState();
    }

    boolean newState_Shoot = true;
    boolean newState_Drive = true;

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        telemetry.addData("State", state);
        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Drive State", driveState);

        switch (state) {
            case SHOOT_PRELOAD:
                updateShoot(new Pose(60.000, 84.000, Math.toRadians(180)), -45);
                break;
            case GO_TO_SHOOT_POS:
                follower.followPath(paths.getBlueCloseStartToShoot(), false);
                if (!follower.isBusy()) {
                    state = getNextState();
                }
                break;
            case MID_PICKUP_GATE:
                updateDrive(paths.getBlueClosePickupSecondMark(), paths.getBlueCloseReturnFromSecondMark(), 0);
                break;

            case SHOOT:
                updateShoot(new Pose(60.000, 84.000, Math.toRadians(180)), -45);
                break;
            case FAR_PICKUP:
                updateDrive(paths.getBlueClosePickupThirdMark(), paths.getBlueCloseReturnFromThirdMark(), 0);
                break;
            case CLOSE_PICKUP:
                updateDrive(paths.getBlueClosePickupFirstMark(), paths.getBlueCloseReturnFromFirstMark(), 0);
        }

        follower.update();

        RobotLog.d("States: " + state + ", " + shootState + ", " + driveState + ", " + newState_Drive);

        telemetry.update();
    }

    private enum ShootState {
        INIT,
        PREPARING,
        SHOOTING_0,
        FINISHED
    }

    ShootState shootState = ShootState.INIT;

    private void updateShoot(Pose holdPose, double turretAngle) {
        if (newState_Shoot) {
            shootState = ShootState.INIT;
        }

        switch (shootState) {
            case INIT:
                robot.getLauncher().setAutoVelocity(1280);
                robot.getLauncher().autoUpdateTurretPID(turretAngle);

                follower.holdPoint(holdPose);

                robot.setRobotState(Robot.RobotInOutStates.OUTTAKE);

                robot.getIndexer().autoFillColorArray();

                if (robot.getLauncher().getLauncherVelocity() >= 1260 && Math.abs(robot.getLauncher().getTurretDegrees() - turretAngle) < 5) {
                    shootState = ShootState.SHOOTING_0;
                }
                else {
                    shootState = ShootState.PREPARING;
                }
                break;
            case PREPARING:

                telemetry.addData("Launcher Velocity", robot.getLauncher().getLauncherVelocity());
                telemetry.addData("Turret Angle", robot.getLauncher().getTurretDegrees());

                robot.getLauncher().autoUpdateTurretPID(turretAngle);
                if (robot.getLauncher().getLauncherVelocity() >= 1260 && Math.abs(robot.getLauncher().getTurretDegrees() - turretAngle) < 5) {
                    shootState = ShootState.SHOOTING_0;
                }
                break;
            case SHOOTING_0:
                robot.getLauncher().autoUpdateTurretPID(turretAngle);
                robot.shootAllBallsAuto();

                if (robot.isNoArtifacts()) {
                    shootState = ShootState.FINISHED;
                }
                break;
            case FINISHED:
                robot.getLauncher().setAutoVelocity(0);
                telemetry.addData("Shoot State", shootState);
                state = getNextState();
                newState_Shoot = true;
        }

        // TODO
        if (shootState != ShootState.FINISHED) {
            newState_Shoot = false;
        }



    }

    private enum DriveState {
        INIT,
        PREPARE,
        PICKUP_1,
        PICKUP_2,
        PICKUP_0,
        PICKUP_2_RETURN,
        PREP_FOR_SHOOT_INIT,
        PREP_FOR_SHOOT,
        FINISHED
    }

    DriveState driveState = DriveState.INIT;

    ElapsedTime delayTimer = new ElapsedTime();

    private boolean resetTimer;

    private void updateDrive(PathChain pickup, PathChain returnToPos, long midDelay) {
        if (newState_Drive) {
            driveState = DriveState.INIT;
        }

        switch (driveState) {
            case INIT:
                follower.followPath(pickup, 0.8, false);
                resetTimer = false;
                robot.getIntake().setIntakeMotorPower(1);
                robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE);
                driveState = DriveState.PREPARE;
                break;
            case PREPARE:
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE, 0.05)) {
                    driveState = DriveState.PICKUP_0;
                }

                if (!follower.isBusy()) {
                    if (midDelay == 0) {
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                    else if (!resetTimer) {
                        delayTimer.reset();
                        resetTimer = true;
                    }
                    else if (delayTimer.milliseconds() >= midDelay){
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                }

                break;
            case PICKUP_0:
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE, 0.05)
                    && robot.getIndexer().isBallAtIntake()) {

                    driveState = DriveState.PICKUP_1;
                    robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE);
                }

                if (!follower.isBusy()) {
                    if (midDelay == 0) {
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                    else if (!resetTimer) {
                        delayTimer.reset();
                        resetTimer = true;
                    }
                    else if (delayTimer.milliseconds() >= midDelay){
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                }

                break;
            case PICKUP_1:
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE, 0.05) &&
                    robot.getIndexer().isBallAtIntake()) {

                    driveState = DriveState.PICKUP_2;
                    robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE);
                }
                if (!follower.isBusy()) {
                    if (midDelay == 0) {
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                    else if (!resetTimer) {
                        delayTimer.reset();
                        resetTimer = true;
                    }
                    else if (delayTimer.milliseconds() >= midDelay){
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                }
                break;
            case PICKUP_2:
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE, 0.05)
                    && robot.getIndexer().isBallAtIntake()) {

                    driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
                    robot.getLauncher().setAutoVelocity(1280);

                }
                if (!follower.isBusy()) {
                    if (midDelay == 0) {
                        driveState = DriveState.PICKUP_2_RETURN;
                    }
                    else if (!resetTimer) {
                        delayTimer.reset();
                        resetTimer = true;
                    }
                    else if (delayTimer.milliseconds() >= midDelay){
                        driveState = DriveState.PICKUP_2_RETURN;
                    }
                }
                break;
            case PICKUP_2_RETURN:
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE, 0.05)
                    && robot.getIndexer().isBallAtIntake()) {

                    driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
                    robot.getLauncher().setAutoVelocity(1280);

                }

                if (!follower.isBusy()) {
                    follower.followPath(returnToPos);
                    driveState = DriveState.FINISHED;
                }
                break;
            case PREP_FOR_SHOOT_INIT:
                driveState = DriveState.PREP_FOR_SHOOT;
                robot.getLauncher().setAutoVelocity(1280);
                robot.getIntake().setIntakeMotorPower(0);
                break;
            case PREP_FOR_SHOOT:

                if (!follower.isBusy()) {
                    driveState = DriveState.FINISHED;
                }

                robot.getLauncher().autoUpdateTurretPID(-45);


                break;
            case FINISHED:
                state = getNextState();
                newState_Drive = true;
                break;
        }

        if (driveState != DriveState.FINISHED) {
            newState_Drive = false;
        }
    }
}
