package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

public class RaceTask extends Task {
    Task[] tasks;
    public RaceTask(Task... tasks) {
        this.tasks = tasks;
    }


    @Override
    public void init() {
        for (Task task : tasks) {
            task.init();
        }
    }

    @Override
    public boolean run() {
        for (int i = 0; i < tasks.length; i++) {
            if (tasks[i].run()) {
                return true;
            }
        }
        return false;
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder toReturn = new StringBuilder();
        toReturn.append("Race Task {\n");
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
