package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

@TeleOp
public class Heading extends OpMode {
    Drivetrain drive = new Drivetrain();

    @Override
    public void init() {
        drive.init(hardwareMap);
        drive.resetYaw();
    }

    @Override
    public void loop () {
        telemetry.addData("Heading from drive.getheading()", drive.getHeading());
        telemetry.update();
    }
}
