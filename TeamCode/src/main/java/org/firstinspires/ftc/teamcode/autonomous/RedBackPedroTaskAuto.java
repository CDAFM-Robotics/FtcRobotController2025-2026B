package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonomous.tasks.AutoTaskMaker;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;
import org.firstinspires.ftc.teamcode.common.util.TelemetrySelector;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.commands.Paths;
import org.firstinspires.ftc.teamcode.tasks.DeadlineTask;
import org.firstinspires.ftc.teamcode.tasks.FollowPathTask;
import org.firstinspires.ftc.teamcode.tasks.HoldPointTask;
import org.firstinspires.ftc.teamcode.tasks.InstantTask;
import org.firstinspires.ftc.teamcode.tasks.NullTask;
import org.firstinspires.ftc.teamcode.tasks.ParallelTask;
import org.firstinspires.ftc.teamcode.tasks.RepeatTask;
import org.firstinspires.ftc.teamcode.tasks.SequentialTask;
import org.firstinspires.ftc.teamcode.tasks.SleepTask;
import org.firstinspires.ftc.teamcode.tasks.Task;
import org.firstinspires.ftc.teamcode.tasks.TaskMaster;

@Autonomous(name = "Red Back", group = "0Competition")
public class RedBackPedroTaskAuto extends OpMode {
    TaskMaster taskMaster;

    Follower follower;
    Robot robot;

    Paths paths;
    AutoTaskMaker taskMaker;

    TelemetrySelector telemetrySelector;

    @Override
    public void init() {

        RobotLog.d("Task: Start Program");
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, true);

        paths = new Paths(follower);

        follower.setStartingPose(new Pose(84, 8.5, Math.toRadians(90)));
        follower.update();

        taskMaker = new AutoTaskMaker(robot, follower);

        telemetrySelector = new TelemetrySelector(telemetry);

        telemetrySelector.addLine("Third Mark", 2, 1);
        telemetrySelector.addLine("Second Mark", 2, 0);
        telemetrySelector.addLine("Loading Zone", 2, 0);
        telemetrySelector.addLine("Loading Zone Repeat", 2, 1);


        robot.getLauncher().setLimelightPipeline(Robot.LLPipelines.OBELISK.ordinal());

    }

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    RobotStaticValuesClass.Obelisk motif;


    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        telemetrySelector.setInput(currentGamepad1.dpad_up && !previousGamepad1.dpad_up, currentGamepad1.dpad_down && !previousGamepad1.dpad_down, currentGamepad1.dpad_right && !previousGamepad1.dpad_right, currentGamepad1.dpad_left && !previousGamepad1.dpad_right);
        telemetrySelector.update();

        motif = robot.getLauncher().getObelisk(robot.isRedSide());

        telemetry.addData("Motif", motif);

        telemetry.update();
    }

    @Override
    public void start() {

        RobotStaticValuesClass.savedObelisk = motif;

        Task autoTask = new NullTask();

        autoTask = autoTask.append(taskMaker.runShootSequenceForObeliskTask(new Pose(84, 8.5, Math.toRadians(90)), AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.RED, 0));


        if (telemetrySelector.getBool(0)) {
            autoTask = autoTask.append(taskMaker.runPickupSequenceTask(paths.getRedFarPickupThirdMark(), paths.getRedFarReturnFromThirdMark(), 500, AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE))
                .append(taskMaker.runShootSequenceForObeliskTask(new Pose(84, 14, Math.toRadians(0)), AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.RED, 0));
        }
        if (telemetrySelector.getBool(1)) {

        }
        if (telemetrySelector.getBool(3)) {
            autoTask = autoTask.append(new RepeatTask(() -> new SequentialTask(
                taskMaker.runPickupSequenceTask(paths.getRedFarPickupHumanPlayerZone(), paths.getRedFarReturnFromHumanPlayerZone(), 1000, AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE),
                taskMaker.runShootSequenceForObeliskTask(new Pose(84, 14, Math.toRadians(0)), AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.RED, -1)
            )));
        }
        else {
            if (telemetrySelector.getBool(2)) {
                autoTask = autoTask.append(taskMaker.runPickupSequenceTask(paths.getRedFarPickupHumanPlayerZone(), paths.getRedFarReturnFromHumanPlayerZone(), 1000, AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE))
                    .append(taskMaker.runShootSequenceForObeliskTask(new Pose(84, 14, Math.toRadians(0)), AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.RED, -1));
            }

            autoTask = autoTask.append(new ParallelTask(
                taskMaker.setLauncherMotorVelocityTask(0),
                taskMaker.stopIntakeTask(),
                new InstantTask(() -> robot.getLauncher().setTurretPower0())
            ));
        }

        autoTask = autoTask.addDeadline(new SleepTask(27000))
            .append(new ParallelTask(
                new SequentialTask(
                    new FollowPathTask(follower, follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), new Pose(108, 36))).setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
                    ),
                    new HoldPointTask(follower, new Pose(108, 36, Math.toRadians(0)))
                ),
                taskMaker.setLauncherMotorVelocityTask(0),
                taskMaker.stopIntakeTask(),
                taskMaker.stopTurretTask()
            ));


        taskMaster = new TaskMaster(autoTask);
    }

    long lastYawRead = Long.MIN_VALUE;

    @Override
    public void loop() {
        try {
            robot.clearBulkCache();

            taskMaster.update();

            robot.getLauncher().updateElevator();

            long currentTime = System.currentTimeMillis();

            if (currentTime - lastYawRead > 250) {
                double yawScalar = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint().getYawScalar();
                lastYawRead = currentTime;
                if (yawScalar != Robot.PINPOINT_B1_YAW_SCALAR) {
                    ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint().setYawScalar(Robot.PINPOINT_B1_YAW_SCALAR);
                }
            }

            follower.update();
            telemetry.addData("Status", taskMaster.getStatus());

            telemetry.update();
        }
        finally {
            RobotStaticValuesClass.autoCompleted = true;
            RobotStaticValuesClass.saveState(
                robot.getDriveBase().getPinPointPose(),
                robot.getLauncher().getLastAngleOffset(),
                motif
            );
        }
    }
}
