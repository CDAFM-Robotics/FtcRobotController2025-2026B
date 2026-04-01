package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonomous.tasks.AutoTaskMaker;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.commands.Paths;
import org.firstinspires.ftc.teamcode.tasks.DeadlineTask;
import org.firstinspires.ftc.teamcode.tasks.FollowPathTask;
import org.firstinspires.ftc.teamcode.tasks.HoldPointTask;
import org.firstinspires.ftc.teamcode.tasks.InstantTask;
import org.firstinspires.ftc.teamcode.tasks.ParallelTask;
import org.firstinspires.ftc.teamcode.tasks.RepeatTask;
import org.firstinspires.ftc.teamcode.tasks.SequentialTask;
import org.firstinspires.ftc.teamcode.tasks.SleepTask;
import org.firstinspires.ftc.teamcode.tasks.TaskMaster;

@Autonomous(name = "Red Back", group = "0Competition")
public class RedBackPedroTaskAuto extends OpMode {
    TaskMaster taskMaster;

    Follower follower;
    Robot robot;

    Paths paths;
    AutoTaskMaker taskMaker;

    @Override
    public void init() {

        RobotLog.d("Task: Start Program");
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, true);

        paths = new Paths(follower);

        follower.setStartingPose(new Pose(84, 8.5, Math.toRadians(90)));
        follower.update();

        taskMaker = new AutoTaskMaker(robot, follower);

        taskMaster = new TaskMaster(new SequentialTask(
            new DeadlineTask(
                new SleepTask(27000),
                new SequentialTask(
                    taskMaker.runShootSequenceTask(new Pose(84, 8.5, Math.toRadians(90)), AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.RED),
                    taskMaker.runPickupSequenceTask(paths.getRedFarPickupThirdMark(), paths.getRedFarReturnFromThirdMark(), 250, AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE),
                    new RepeatTask(() -> new SequentialTask(
                        taskMaker.runShootSequenceTask(new Pose(84, 14, Math.toRadians(0)), AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.RED),
                        taskMaker.runPickupSequenceTask(paths.getRedFarPickupHumanPlayerZone(), paths.getRedFarReturnFromHumanPlayerZone(), 1000, AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE)
                    ))
                )
            ),
            new ParallelTask(
                new SequentialTask(
                    new FollowPathTask(follower, follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), new Pose(108, 36))).setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
                    ),
                    new HoldPointTask(follower, new Pose(108, 36, Math.toRadians(0)))
                ),
                taskMaker.setLauncherMotorVelocityTask(0),
                taskMaker.stopIntakeTask(),
                new InstantTask(() -> robot.getLauncher().setTurretPower0())
            )
        ));
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
