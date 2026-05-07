package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

@TeleOp
public class Arm extends OpMode {
    Drivetrain drive = new Drivetrain();
    float position = 0;

    @Override
    public void init() { drive.init(hardwareMap); }

    @Override
    public void loop() {
        telemetry.addData("Welcome to the Arm test for our FRC robot!!", null);
        telemetry.addData("Press on both the triggers to move the arm back and forth!", null);

        if (gamepad1.a) {
            drive.runArm(1);
        }
        if (gamepad1.b) {
            drive.runArm(0);
        }


        telemetry.addData("Arm Position", drive.getArm());
        telemetry.update();
    }
}

