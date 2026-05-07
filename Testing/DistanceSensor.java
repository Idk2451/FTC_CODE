package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Sensors;

@TeleOp
public class DistanceSensor extends OpMode {
   Sensors sensor = new Sensors();

   @Override
   public void init() {
       sensor.init(hardwareMap);
   }

   @Override
    public void loop() {
       telemetry.addData("Distance sensor right: ", sensor.getRobotRight());
       telemetry.addData("Distance sensor left: ", sensor.getRobotLeft());
       telemetry.addData("Distance sensor front: ", sensor.getRobotFront());
       telemetry.addData("Distance sensor back: ", sensor.getRobotBack());
       telemetry.update();
   }
}
