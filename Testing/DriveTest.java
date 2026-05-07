package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Sensors;

@TeleOp
public class DriveTest extends OpMode {
    Drivetrain drive = new Drivetrain();
    Sensors sensor = new Sensors();

    @Override
    public void init() {
        drive.init(hardwareMap);
        sensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Welcome to driving test for our FRC Robot!!", null);

        // Setting variables for driving
        float forward = -gamepad1.left_stick_y;
        float right = gamepad1.left_stick_x;
        float rotate = gamepad1.right_stick_x;

        sensor.tailLights(forward);
        drive.drive(forward, right, rotate);
    }
}
