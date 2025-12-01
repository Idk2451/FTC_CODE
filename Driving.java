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
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

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
        if (gamepad1.left_bumper) {
            drive.runLauncherServos(0.0, 0.85);
        }
        if (gamepad1.right_bumper) {
            drive.runLauncherServos(0.75, 0.15);
        }
      }
    }