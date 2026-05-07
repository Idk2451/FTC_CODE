package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

@TeleOp
public class ServoPosition extends OpMode {
    Drivetrain drive = new Drivetrain();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Arm Position: ", drive.getArm());
        telemetry.update();
    }
}
