package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Sensors;

@Autonomous
public class PushRocket extends OpMode {
    Drivetrain drive = new Drivetrain();
    Sensors sensor = new Sensors();
    ElapsedTime timer1 = new ElapsedTime();
    State state = State.START;
    enum State {
        START,
        STRAFE1,
        PUSHROCKET,
        BACKOUTROCKET,
        STOP
    }

    @Override
    public void init() {
        drive.init(hardwareMap);
        sensor.init(hardwareMap);
        drive.resetYaw();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Heading", drive.getHeading());
        telemetry.update();
    }

    @Override
    public void start() { timer1.reset(); }

    @Override
    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Time Elapsed", timer1);
        switch (state) {
            case START:
                timer1.reset();
                state = State.STRAFE1;
                break;

            case STRAFE1:
                if (sensor.getRobotLeft() < 14.5 ) {
                    timer1.reset();
                    drive.drive(0, 0, 0);
                    state = State.PUSHROCKET;

                }
                else {
                    drive.driveStrafe(-0.2, 0 );
                }
                break;

            case PUSHROCKET:
                if (timer1.milliseconds() > 4900) {
                    timer1.reset();
                    drive.drive(0, 0, 0);
                    state = State.BACKOUTROCKET;
                }
                else {

                    drive.driveStraight(-0.2, 0);
                }
                break;

            case BACKOUTROCKET:
                if (timer1.milliseconds() > 200) {
                    timer1.reset();
                    drive.drive(0, 0,0);
                    state = State.STOP;
                }
                else {
                    drive.driveStraight(0.3, 0);
                }
                break;

            case STOP:
                break;
        }
    }

}