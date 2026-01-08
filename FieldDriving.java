package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldDriving extends OpMode {
    Drivetrain drive = new Drivetrain();
    IMU imu;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }
    private void driveFieldRelative(double forward, double right, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // Convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive.drive(newForward, newRight, rotate);
    }

    @Override
    public void loop() {
        double right_trigger_value = (gamepad1.right_trigger);
        double left_trigger_value = (-gamepad1.left_trigger);
        double speed = left_trigger_value + right_trigger_value;
        drive.runBall_launcher(speed);

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        driveFieldRelative(forward, right, rotate);

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