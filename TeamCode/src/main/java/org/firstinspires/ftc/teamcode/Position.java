package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Position {
    int position;
    ElapsedTime runtime = new ElapsedTime();

    public void setPosition(int pos) {
        position = pos;
    }

    public int getPosition() {

        return position;
    }

    public void startTime() {
        runtime.startTime();
    }

    public double getTime() {
        return runtime.seconds();

    }
    public void resetTime() {
        runtime.reset();
    }

}
