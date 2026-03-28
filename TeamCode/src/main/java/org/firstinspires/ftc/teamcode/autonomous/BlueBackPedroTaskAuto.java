package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.tasks.AutoTaskMaker;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.commands.Paths;
import org.firstinspires.ftc.teamcode.tasks.DeadlineTask;
import org.firstinspires.ftc.teamcode.tasks.FollowPathTask;
import org.firstinspires.ftc.teamcode.tasks.HoldPointTask;
import org.firstinspires.ftc.teamcode.tasks.RepeatTask;
import org.firstinspires.ftc.teamcode.tasks.SequentialTask;
import org.firstinspires.ftc.teamcode.tasks.SleepTask;
import org.firstinspires.ftc.teamcode.tasks.TaskMaster;

@Autonomous(name = "Blue Back", group = "0Competition")
public class BlueBackPedroTaskAuto extends OpMode {
    TaskMaster taskMaster;

    Follower follower;
    Robot robot;

    Paths paths;
    AutoTaskMaker taskMaker;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, false);

        paths = new Paths(follower);

        follower.setStartingPose(new Pose(60, 8.5, Math.toRadians(90)));
        follower.update();

        taskMaker = new AutoTaskMaker(robot, follower);

        taskMaster = new TaskMaster(new SequentialTask(
            new DeadlineTask(
                new SleepTask(27000),
                new SequentialTask(
                    taskMaker.runShootSequenceTask(new Pose(60, 8.5, Math.toRadians(90)), AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE),
                    taskMaker.runPickupSequenceTask(paths.getBlueFarPickupThirdMark(), paths.getBlueFarReturnFromThirdMark(), 250, AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE),
                    new RepeatTask(() -> new SequentialTask(
                        taskMaker.runShootSequenceTask(new Pose(60, 14, Math.toRadians(180)), AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE),
                        taskMaker.runPickupSequenceTask(paths.getBlueFarPickupHumanPlayerZone(), paths.getBlueFarReturnFromHumanPlayerZone(), 1000, AutoTaskMaker.Side.FAR, AutoTaskMaker.Team.BLUE)
                    ))
                )
            ),
            new FollowPathTask(follower, follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(36, 36))).setConstantHeadingInterpolation(Math.toRadians(180))
                .build()
            ),

            new HoldPointTask(follower, new Pose(36, 36, Math.toRadians(180)))
        ));
    }

    @Override
    public void loop() {
        taskMaster.update();

        follower.update();
        telemetry.addData("Status", taskMaster.getStatus());

        telemetry.addData("Task", taskMaster.getTask().toString());

        telemetry.update();
    }
}
