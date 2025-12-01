package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous
public class Autonomus extends OpMode {
    enum State {
        START,
        DRIVE,
        DONE
    }
    Drivetrain drive = new Drivetrain();
    State state = State.START;
    double LastTime;

    @Override
    public void init() {drive.init(hardwareMap);
        resetRuntime();
        LastTime = getRuntime();
    }
    @Override
    public void loop () {
        telemetry.addData( "State", state);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in state", getRuntime() - LastTime);
        switch(state){
            case START:

                drive.runLauncher(1.0);
                drive.runIntake(1.0);
                resetRuntime();
                state = State.DRIVE;
            case DRIVE:
                if (getRuntime() >= 2.0) {
                    state = State.DONE;
                }
                else {
                    drive.driveStraight(1.0, 0);
                }
                break;

            case DONE:
                drive.drive(0,0,0);
        }
    }
}

