package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutonomousTest extends OpMode {
    Webcam webcam = new Webcam();
    State state = State.START;
    enum State {
        START,
        SCAN,
        STOP
    }

    @Override
    public void init() {
        webcam.init(hardwareMap);
        telemetry.addData("If you wanna see a preview of the webcam Press on the 3 dots, and go to Camera Stream.", "You'll thank me later.");
    }

    @Override
    public void loop() {
        telemetry.addData("State", state);
        switch (state) {
            case START:
                state = State.SCAN;
                break;

            case SCAN:
                webcam.GetCameraOrientation();
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",

                break;


            case STOP:
                webcam.stop();
                state = State.STOP;
                break;
        }
    }
}
