package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

public class NullTask extends Task{
    @Override
    public void init() {}

    @Override
    public boolean run() {
        return true;
    }

    @NonNull
    @Override
    public String toString() {
        return "Null Task";
    }


}
