package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain  {
    private IMU imu;
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;
    private DcMotor launch_motor;
    private DcMotor left_intake_motor;
    private DcMotor right_intake_motor;
    private DcMotor ball_launch_motor;
    private Servo left_intake_servo;
    private Servo right_intake_servo;
    private Servo left_sorting_servo;
    private Servo middle_sorting_servo;
    private Servo right_sorting_servo;
    public DistanceSensor front_distance;
    public DistanceSensor back_distance;
    // Variables
    public double P_DRIVE_GAIN;
    public double P_TURN_GAIN;
    public double turnSpeed;
    public void init(@NonNull HardwareMap hardwareMap) {
        // Hardware Mapping
        front_distance = hardwareMap.get(DistanceSensor.class, "front_distance");
        back_distance = hardwareMap.get(DistanceSensor.class, "back_distance");
        left_intake_motor = hardwareMap.dcMotor.get("left_intake_motor");
        right_intake_motor = hardwareMap.dcMotor.get("right_intake_motor");
        ball_launch_motor = hardwareMap.dcMotor.get("ball_launch_motor");
        left_intake_servo = hardwareMap.servo.get("left_intake_servo");
        right_intake_servo = hardwareMap.servo.get("right_intake_servo");
        left_sorting_servo = hardwareMap.servo.get("left_sorting_servo");
        middle_sorting_servo = hardwareMap.servo.get("middle_sorting_servo");
        right_sorting_servo = hardwareMap.servo.get("right_sorting_servo");
        launch_motor = hardwareMap.dcMotor.get("launch_motor");
        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right  = hardwareMap.dcMotor.get("back_right");
        front_right = hardwareMap.dcMotor.get("front_right");
        // IMU HARDWARE MAPPING
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Setting the opposite side as the opposite of the other
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        launch_motor.setDirection(DcMotor.Direction.REVERSE);
        left_intake_motor.setDirection(DcMotor.Direction.FORWARD);
        right_intake_motor.setDirection(DcMotor.Direction.REVERSE);
        ball_launch_motor.setDirection(DcMotor.Direction.FORWARD);
        // Settings the Encoding
        front_left.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ball_launch_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Braking
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ball_launch_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void setPowers(double front_left_power, double back_left_power, double back_right_power , double front_right_power) {
        double maxSpeed  = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(front_left_power));
        maxSpeed = Math.max(maxSpeed, Math.abs(back_left_power));
        maxSpeed = Math.max(maxSpeed, Math.abs(front_right_power));
        maxSpeed = Math.max(maxSpeed, Math.abs(back_right_power));

        front_left_power /= maxSpeed;
        back_left_power /= maxSpeed;
        back_right_power /= maxSpeed;
        front_right_power /= maxSpeed;

        front_left.setPower(front_left_power);
        back_left.setPower(back_left_power);
        front_right.setPower(front_right_power);
        back_right.setPower(back_right_power);
    }
    public void drive (double forward, double right, double rotate) {
        double front_left_power = forward + right + rotate;
        double back_left_power = forward + right - rotate;
        double front_right_power = forward - right + rotate;
        double back_right_power = forward - right - rotate;
        setPowers(front_left_power, back_left_power, front_right_power, back_right_power);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {

        double headingError = getHeading() - desiredHeading;
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return
                orientation.getYaw(AngleUnit.DEGREES);
    }
    public void driveStraight(double forward, double heading) {
        turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
        drive(forward, 0, turnSpeed);
    }
    public void driveStrafe(double right, double heading) {
        turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
        drive(0, right, turnSpeed);
    }
    public void runLauncher (double launchPower) {
        launch_motor.setPower(launchPower);
    }
    public void runIntake (double intakePower) {
         left_intake_motor.setPower(intakePower);
         right_intake_motor.setPower(intakePower);
    }
    public void runIntakeServos (double left_intake_servo_position, double right_intake_servo_position) {
        left_intake_servo.setPosition(left_intake_servo_position);
        right_intake_servo.setPosition(right_intake_servo_position);
    }
    public void runBall_launcher (double ball_launcher_power) { ball_launch_motor.setPower(ball_launcher_power); }
    public void runSorterServos (double left_sorting_servo_position, double middle_sorting_servo_position, double right_sorting_servo_position) {
        left_sorting_servo.setPosition(left_sorting_servo_position);
        middle_sorting_servo.setPosition(middle_sorting_servo_position);
        right_sorting_servo.setPosition(right_sorting_servo_position);
    }
}
