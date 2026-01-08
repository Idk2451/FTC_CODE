package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Timer;

@Autonomous
public class Autonomus extends OpMode {
    public int shoot_line = 100;
    enum State {
        START,
        DRIVE,
    }
    Drivetrain drive = new Drivetrain();
    State state = State.START;
    @Override
    public void init() {
    }
    public void start() {
        State state = State.DRIVE;
    }
    @Override
    public void loop () {
        telemetry.addData( "State", state);
        switch(state){
            case DRIVE:
                if (drive.front_distance.getDistance(DistanceUnit.CM) > shoot_line) {
                    drive.driveStraight(1, 0);
                }
                else {
                    break;
                }
        }
    }
}

