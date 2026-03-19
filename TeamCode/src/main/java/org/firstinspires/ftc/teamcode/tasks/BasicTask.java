package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

import java.util.function.Supplier;

public class BasicTask extends Task{
    Runnable onInit;
    Supplier<Boolean> onRun;

    public BasicTask(Runnable init, Supplier<Boolean> run) {
        onInit = init;
        onRun = run;
    }


    @Override
    public void init() {
        onInit.run();
    }

    @Override
    public boolean run() {
        return onRun.get();
    }

    @NonNull
    @Override
    public String toString() {
        return "Basic Task: " + onInit.toString() + ", " + onRun.toString();
    }


}
