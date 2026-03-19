package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

import java.util.function.Supplier;

/**
 * WARNING:
 * This task has no termination condition. To terminate it you have to use another Task
 */

public class RepeatTask extends Task{

    Supplier<Task> taskSupplier;
    Task currentTask;

    public RepeatTask(Supplier<Task> taskSupplier) {
        this.taskSupplier = taskSupplier;
    }

    @Override
    public void init() {
        currentTask = taskSupplier.get();
        currentTask.init();
    }

    @Override
    public boolean run() {
        if (currentTask.run()) {
            init();
        }

        return false;
    }

    @NonNull
    @Override
    public String toString() {
        return "Repeat Task: " + taskSupplier.toString();
    }
}
