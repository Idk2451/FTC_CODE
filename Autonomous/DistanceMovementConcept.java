package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Sensors;

@Autonomous
public class DistanceMovementConcept extends OpMode {
    Drivetrain drive = new Drivetrain();
    Sensors sensor = new Sensors();
    ElapsedTime timer1 = new ElapsedTime();
    State state = State.START;
    enum State {
        START,
        MOVE1,
        TURN1,
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
    public void start() { timer1.reset();}

    @Override
    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Time Elapsed", timer1);
        switch (state) {
            case START:
                timer1.reset();
                state = State.MOVE1;
                break;

            case MOVE1:
                if (sensor.getRobotFront() < 200) {
                    timer1.reset();
                    drive.drive(0, 0, 0);
                    state = State.TURN1;
                }
                else {
                    drive.driveStraight(1, 0);
                }
                break;

            case TURN1:
                if (drive.getHeading() > 95) {
                    drive.drive(0, 0, 0);
                    state = State.STOP;
                }
                else {
                    drive.turnToHeading(1, 95);
                }
                break;

            case STOP:
                break;
        }
    }
}
