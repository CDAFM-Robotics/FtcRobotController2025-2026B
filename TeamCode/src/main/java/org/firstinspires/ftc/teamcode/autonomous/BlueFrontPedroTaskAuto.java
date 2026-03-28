package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

@Autonomous(name = "Blue Front", group = "0Competition")
public class BlueFrontPedroTaskAuto extends OpMode {
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

        follower.setStartingPose(new Pose(18, 120, Math.toRadians(54)));
        follower.update();

        taskMaker = new AutoTaskMaker(robot, follower);

        taskMaster = new TaskMaster(new SequentialTask(
            new DeadlineTask(
                new SleepTask(27000),
                new SequentialTask(
                    taskMaker.runShootSequenceTask(new Pose(60, 84, Math.toRadians(90)), AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.BLUE),
                    taskMaker.runPickupSequenceTask(paths.getBlueClosePickupSecondMark(), paths.getBlueCloseReturnFromSecondMark(), 250, AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.BLUE),
                    taskMaker.runShootSequenceTask(new Pose(60, 84, Math.toRadians(90)), AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.BLUE),
                    taskMaker.runPickupSequenceTask(paths.getBlueClosePickupFirstMark(), paths.getBlueCloseReturnFromFirstMark(), 250, AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.BLUE),
                    taskMaker.runShootSequenceTask(new Pose(60, 84, Math.toRadians(90)), AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.BLUE),
                    taskMaker.runPickupSequenceTask(paths.getBlueClosePickupThirdMark(), paths.getBlueCloseReturnFromThirdMark(), 250, AutoTaskMaker.Side.NEAR, AutoTaskMaker.Team.BLUE)
                )
            ),
            new FollowPathTask(follower, follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(54, 126))).setConstantHeadingInterpolation(Math.toRadians(180))
                .build()
            ),

            new HoldPointTask(follower, new Pose(36, 126, Math.toRadians(180)))
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
