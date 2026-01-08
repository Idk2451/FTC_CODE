package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class Driving extends OpMode {
    Drivetrain drive = new Drivetrain();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double right_trigger_value = (gamepad1.right_trigger);
        double left_trigger_value = (-gamepad1.left_trigger);
        double speed = left_trigger_value + right_trigger_value;
        drive.runBall_launcher(speed);

        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_x;
        double rotate = -gamepad1.left_stick_x;

        drive.drive(forward, right, rotate);

        if (gamepad1.y) {
            drive.runLauncher(1.0);
        }
        if (gamepad1.a) {
            drive.runLauncher(0);
        }
        if (gamepad1.x) {
            drive.runIntake(1.0);
            drive.runIntakeServos(1.0, 0.0);
        }
        if (gamepad1.b) {
            drive.runIntake(0);
            drive.runIntakeServos(0.5, 0.5);
        }
        if (gamepad1.dpad_left) {
            drive.runSorterServos(0, 0, 0);
        }
        if (gamepad1.dpad_right) {
            drive.runSorterServos(1, 1, 1);
        }
        if (gamepad1.dpad_up) {
            drive.runSorterServos(1, 0.5, 0);
        }
        if (gamepad1.dpad_down) {
            drive.runSorterServos(0.5, 0.5, 0.5);
        }
    }
}