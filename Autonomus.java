package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.concurrent.TimeUnit;

@Autonomous
public class Autonomus extends OpMode {
    Drivetrain drive = new Drivetrain();
    State state = State.START;
    enum State {
        START,
        DRIVE,
        SHOOT,
        STOP
    }
    @Override
    public void init() {
        drive.init(hardwareMap);
    }
    @Override
    public void loop () {
        telemetry.addData( "State", state);
        switch(state){
            case START:
                drive.runLauncher(1.0);
                drive.runIntake(1.0);
                drive.runIntakeServos(1.0, 0.0);
                state = State.DRIVE;
                break;

            case DRIVE:
                drive.driveStraight(1.0, 0.0);
                try {
                    TimeUnit.SECONDS.sleep(2);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
            }
                drive.driveStraight(0.0,0.0);
                drive.drive(0.0, 0.0, 0.5);
                state = State.SHOOT;
                break;
            case SHOOT:
                drive.runBall_launcher(1);
                state = State.STOP;
            case STOP:
                drive.runBall_launcher(0);
                drive.runLauncher(0);
                drive.runIntake(0);
                drive.runIntakeServos(0.5, 0.5);
                break;
        }
    }
}

