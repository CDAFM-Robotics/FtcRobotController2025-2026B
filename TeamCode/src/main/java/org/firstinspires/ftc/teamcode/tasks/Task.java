package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

public abstract class Task {

    abstract public void init();

    abstract public boolean run();

    @NonNull
    abstract public String toString();
}
