package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class TelemetrySelector {

    private Telemetry telemetry;

    private int lines;

    private HashMap<Integer, String> captions = new HashMap<>();
    private HashMap<Integer, Selection> selections = new HashMap<>();

    private Selection lineSelection;



    public TelemetrySelector(Telemetry telemetry) {
        this.telemetry = telemetry;
        lineSelection = new Selection(0);
    }

    public void addLine(String caption, int range) {
        captions.put(lines, caption);
        selections.put(lines, new Selection(range));
        lineSelection.setRange(++lines);
    }

    public void setInput(boolean up, boolean down, boolean inc, boolean dec) {
        if (up) {
            lineUp();
        }
        if (down) {
            lineDown();
        }
        if (inc) {
            increment();
        }
        if (dec) {
            decrement();
        }
    }

    private void lineUp() {
        lineSelection.decrement();
    }
    private void lineDown() {
        lineSelection.increment();
    }
    private void increment() {
        selections.get(lineSelection.value).increment();
    }
    private void decrement() {
        selections.get(lineSelection.value).decrement();
    }


    public void update() {
        for (int i = 0; i < lines; i++) {
            if (i == lineSelection.getValue()) {
                telemetry.addData("> " + captions.get(i), selections.get(i).getValue() +  ", " + selections.get(i).getBoolValue());
            }
            else {
                telemetry.addData("   " + captions.get(i), selections.get(i).getValue() +  ", " + selections.get(i).getBoolValue());
            }
        }
    }

    public int get(int index) {
        return selections.get(index).getValue();
    }

    public boolean getBool(int index) {
        return selections.get(index).getBoolValue();
    }
}
