package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

public class DeadlineTask extends Task{
    boolean[] status;
    Task deadline;
    Task[] tasks;
    public DeadlineTask(Task deadline, Task... tasks) {
        this.deadline = deadline;
        this.tasks = tasks;

        status = new boolean[tasks.length];
    }


    @Override
    public void init() {
        for (Task task : tasks) {
            task.init();
        }
        deadline.init();
    }

    @Override
    public boolean run() {
        for (int i = 0; i < tasks.length; i++) {
            if (status[i] || tasks[i].run()) {
                status[i] = true;
            }
        }
        return deadline.run();
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder toReturn = new StringBuilder();
        toReturn.append("Deadline Task {\n");
        toReturn.append(deadline.toString());
        for (Task task : tasks) {
            toReturn.append(", ");
            toReturn.append(task.toString());
        }
        return toReturn + "\n}";
    }
}
