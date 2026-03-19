package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class FollowPathTask extends Task{
    Follower follower;
    PathChain path;
    public FollowPathTask(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.path = pathChain;
    }

    @Override
    public void init() {
        follower.followPath(path, 1, false);
    }

    @Override
    public boolean run() {
        return !follower.isBusy();
    }

    @NonNull
    @Override
    public String toString() {
        return "Follow Path Task: " + path.toString();
    }


}
