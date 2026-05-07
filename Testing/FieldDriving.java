package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Sensors;

@TeleOp
public class FieldDriving extends OpMode {
    Drivetrain drive = new Drivetrain();
    Sensors sensor = new Sensors();

    @Override
    public void init() {
        drive.init(hardwareMap);
        sensor.init(hardwareMap);
        drive.resetYaw();
    }

    @Override
    public void loop() {
        telemetry.addData("Welcome to Field Driving!", null);
        telemetry.addData("Heading", drive.getHeading());
        telemetry.update();

        float forward = -gamepad1.left_stick_y;
        float right = gamepad1.left_stick_x;
        float rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, right, rotate);
    }
}
