package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueAuto extends OpMode {
    Drivetrain drive = new Drivetrain();
    ElapsedTime timer1 = new ElapsedTime();
    State state = State.START;
    enum State {
        START,
        TURN1,
        SHOOT,
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

    @Override
    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Time Elapsed", timer1);
        switch (state) {
            case START:
                if (timer1.milliseconds() >= 2000) {
                    timer1.reset();
                    state = State.TURN1;
                }
                else {
                    drive.runLauncher(2000);
                }
                break;
            case TURN1:
                if (timer1.seconds() >= 5) {
                    timer1.reset();
                    drive.drive(0, 0, 0);
                    state = State.SHOOT;
                }

                else {
                    drive.turnToHeading(1.0, -68);
                }
                break;
            case SHOOT:
                if (timer1.seconds() >= 10) {
                    drive.runBall_launcher(0);
                    drive.runSorterServos(0.5, 0.5, 0.5);
                    timer1.reset();
                    state = State.DRIVE;
                }
                else {
                    drive.runBall_launcher(1.0);
                    drive.runSorterServos(1,0.5, 0);
                }
                break;
            case DRIVE:
                if (timer1.milliseconds() >= 1100) {
                    timer1.reset();
                    state = State.STOP;
                }
                else {
                    drive.driveStraight(-1.0, 0);
                }
                break;
            case STOP:
                drive.runLauncher(0);
                drive.driveStraight(0, 0);
                break;
            }
        }
    }

