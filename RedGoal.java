package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedGoal extends OpMode {
    Drivetrain drive = new Drivetrain();
    ElapsedTime timer1 = new ElapsedTime();
    State state = State.DRIVE;

    enum State {
        DRIVE,
        STOP
    }

    @Override
    public void init() {
        drive.init(hardwareMap);
        drive.imu.resetYaw();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Heading", drive.getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        timer1.reset();
    }

    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Time Elapsed", timer1);
        switch (state) {
            case DRIVE:
                if (timer1.milliseconds() >= 900) {
                    state = State.STOP;

                }
                else {
                drive.driveStraight(1.0, 90);
                }
                break;

            case STOP:
                drive.driveStraight(0, 90);
                break;
        }
    }
}

