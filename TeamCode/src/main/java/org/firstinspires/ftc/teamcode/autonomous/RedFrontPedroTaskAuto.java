package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonomous.tasks.AutoTaskMaker;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;
import org.firstinspires.ftc.teamcode.common.util.TelemetrySelector;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.commands.Paths;
import org.firstinspires.ftc.teamcode.tasks.DeadlineTask;
import org.firstinspires.ftc.teamcode.tasks.FollowPathTask;
import org.firstinspires.ftc.teamcode.tasks.HoldPointTask;
import org.firstinspires.ftc.teamcode.tasks.InstantTask;
import org.firstinspires.ftc.teamcode.tasks.NullTask;
import org.firstinspires.ftc.teamcode.tasks.ParallelTask;
import org.firstinspires.ftc.teamcode.tasks.SequentialTask;
import org.firstinspires.ftc.teamcode.tasks.SleepTask;
import org.firstinspires.ftc.teamcode.tasks.Task;
import org.firstinspires.ftc.teamcode.tasks.TaskMaster;

@Autonomous(name = "Red Front", group = "0Competition")
public class RedFrontPedroTaskAuto extends OpMode {
    TaskMaster taskMaster;

    Follower follower;
    Robot robot;

    Paths paths;
    AutoTaskMaker taskMaker;

    TelemetrySelector telemetrySelector;

    @Override
    public void init() {

        DebugManager.TELEMETRY_ENABLED = false;
        DebugManager.ROBOT_LOG_ENABLED = true;
        DebugManager.LOG_DRIVEBASE  = false;
        DebugManager.LOG_PINPOINT   = false;
        DebugManager.LOG_VISION     = false;
        DebugManager.LOG_LAUNCHER   = false;
        DebugManager.LOG_SPINDEXER  = false;
        DebugManager.LOG_INTAKE     = false;
        DebugManager.LOG_HUD        = false;
        DebugManager.LOG_ROBOT      = false;


        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, true);

        paths = new Paths(follower);

        follower.setStartingPose(new Pose(126, 120, Math.toRadians(126)));
        follower.update();

        taskMaker = new AutoTaskMaker(robot, follower);

        telemetrySelector = new TelemetrySelector(telemetry);

        telemetrySelector.addLine("Second Mark", 2);
        telemetrySelector.addLine("Gate on Second Mark", 2);
        telemetrySelector.addLine("First Mark", 2);
        telemetrySelector.addLine("Third Mark", 2);
    }

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        telemetrySelector.setInput(currentGamepad1.dpad_up && !previousGamepad1.dpad_up, currentGamepad1.dpad_down && !previousGamepad1.dpad_down, currentGamepad1.dpad_right && !previousGamepad1.dpad_right, currentGamepad1.dpad_left && !previousGamepad1.dpad_right);
        telemetrySelector.update();
    }

    @Override
    public void start() {
        Task autoTask = new NullTask();
        autoTask = autoTask.append(new DeadlineTask(
                new FollowPathTask(follower, paths.getRedCloseStartToShoot2()),
                taskMaker.setLauncherToGoalTask(),
                taskMaker.setCloseLauncherTask()
            ))
            .append(taskMaker.runShootSequenceTask(new Pose(144 - 60, 84, Math.toRadians(90)), AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.RED));

        if (telemetrySelector.getBool(0)) {
            if (telemetrySelector.getBool(1)) {

            }
            else {
                autoTask = autoTask.append(taskMaker.runPickupSequenceTask(paths.getRedClosePickupSecondMark(), paths.getRedCloseReturnFromSecondMark(), 500, AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.RED))
                    .append(taskMaker.runShootSequenceTask(new Pose(144 - 60, 84, Math.toRadians(0)), AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.RED));
            }
        }

        if (telemetrySelector.getBool(2)) {
            autoTask = autoTask.append(taskMaker.runPickupSequenceTask(paths.getRedClosePickupFirstMark(), paths.getRedCloseReturnFromFirstMark(), 500, AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.RED))
                .append(taskMaker.runShootSequenceTask(new Pose(144 - 60, 84, Math.toRadians(0)), AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.RED));
        }
        if (telemetrySelector.getBool(3)) {
            autoTask = autoTask.append(taskMaker.runPickupSequenceTask(paths.getRedClosePickupThirdMark(), paths.getRedCloseReturnFromThirdMark(), 500, AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.RED))
                .append(taskMaker.runShootSequenceTask(new Pose(144 - 60, 84, Math.toRadians(0)), AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.RED));
        }

        autoTask = autoTask.append(new ParallelTask(
                taskMaker.setLauncherMotorVelocityTask(0),
                taskMaker.stopIntakeTask(),
                new InstantTask(() -> robot.getLauncher().setTurretPower0())
            ))
            .addDeadline(new SleepTask(27000))
            .append(new ParallelTask(
                new SequentialTask(
                    new FollowPathTask(follower, follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), new Pose(90, 126))).setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
                    ),
                    new HoldPointTask(follower, new Pose(90, 126, Math.toRadians(0)))
                ),
                taskMaker.setLauncherMotorVelocityTask(0),
                taskMaker.stopIntakeTask(),
                taskMaker.stopTurretTask()
            ));

        RobotLog.d("Task: " + autoTask.toString());


        taskMaster = new TaskMaster(autoTask);
    }

    @Override
    public void loop() {
        taskMaster.update();

        robot.getLauncher().updateElevator();

        if (((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint().getYawScalar() != Robot.PINPOINT_B1_YAW_SCALAR) {
            ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint().setYawScalar(Robot.PINPOINT_B1_YAW_SCALAR);
        }

        follower.update();
        telemetry.addData("Status", taskMaster.getStatus());

        telemetry.update();
    }
}
