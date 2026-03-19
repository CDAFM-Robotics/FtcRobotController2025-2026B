package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

public class ParallelTask extends Task{
    boolean[] status;
    Task[] tasks;
    public ParallelTask(Task... tasks) {
        this.tasks = tasks;
        status = new boolean[tasks.length];
    }


    @Override
    public void init() {
        for (Task task : tasks) {
            task.init();
        }
    }

    @Override
    public boolean run() {
        boolean toReturn = true;
        for (int i = 0; i < tasks.length; i++) {
            if (!status[i] && !tasks[i].run()) {
                toReturn = false;
            }
            else {
                status[i] = true;
            }
        }
        return toReturn;
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder toReturn = new StringBuilder();
        toReturn.append("Sequential Task {\n");
        boolean firstLoop = true;
        for (Task task : tasks) {
            if (!firstLoop) {
                toReturn.append(", ");
            }
            firstLoop = false;
            toReturn.append(task.toString());
        }
        return toReturn + "\n}";
    }
}
