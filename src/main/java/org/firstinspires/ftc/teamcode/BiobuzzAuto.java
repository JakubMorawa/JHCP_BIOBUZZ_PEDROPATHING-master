package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "BiobuzzAuto", group = "Robot")

public class BiobuzzAuto extends LinearOpMode {


    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor left_back = null;
    private DcMotor right_back = null;

    private DcMotor shooterMotor = null;
    private DcMotor intakeMotor = null;
    private DcMotor elevatorMotor = null;

    private Servo tunnelServo = null;
    
    boolean shooterMotorstate = false;
    boolean intakeMotorstate = false;
    boolean elevatorMotorstate = false;

    boolean tunnelState = false;
    boolean fanState = false;
    
    boolean prevStateA = false;
    boolean prevStateX = false;
    boolean prevStateY = false;
    boolean prevStateB = false;
    boolean prevStatRightBumper = false;

    double shooterMotorPower = 0;
    double elevatorPower = 0;



    private final double FORWARD_SPEED = 0.5;

    private ElapsedTime runtime = new ElapsedTime();


    // ----------------- Helper Methods -----------------


    private void move(double x, double y, double rx, double duration) {

        double leftFrontPower = (y - x + r)*FORWARD_SPEED;
        double leftBackPower = (y + x + r)*FORWARD_SPEED;
        double rightFrontPower = (y + x - r)*FORWARD_SPEED;
        double rightBackPower = (y - x - r)*FORWARD_SPEED;

        double magnitude = 1.0;
        magnitude = Math.max(magnitude, Math.abs(leftFrontPower));
        magnitude = Math.max(magnitude, Math.abs(leftBackPower));
        magnitude = Math.max(magnitude, Math.abs(rightFrontPower));
        magnitude = Math.max(magnitude, Math.abs(rightBackPower));

        left_front.setPower(leftFrontPower / magnitude);
        left_back.setPower(leftBackPower / magnitude);
        right_front.setPower(rightFrontPower / magnitude);
        right_back.setPower(rightBackPower / magnitude);

    }


    private void moveAtAngle(double angleDegrees, double speed, double duration) {

        double radians = Math.toRadians(angleDegrees);
        double y = -Math.cos(radians); // forward component
        double x = Math.sin(radians);  // strafe component
        
        // call your existing move() function with calculated x/y

        move(0, y, x, duration);

    }


    private void stopDriveMotors() {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }


    private void setTunnelPosition(double tunnelPosition, double waitDuration) {
        tunnelServo.setPosition(tunnelPosition);
        delay(waitDuration);
    }


    private void delay(double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime .seconds() < seconds) {
            // Keep looping without blocking
        }
    }


    private void shooterMotorOn(boolean state, double power) {
        shooterMotor.setPower(state ? -1*power : 0);
    }


    private void elevatorOn(boolean state) {
        elevatorMotor.setPower(state ? -1 : 0);
    }


    private void intakeMotorOn(boolean state) {
        intakeMotor.setPower(state ? -1 : 0);
    }

    // ----------------- Main Autonomous Routine -----------------


    @Override
    public void runOpMode() {

        // Initialize hardware

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");

        tunnelServo = hardwareMap.get(Servo.class, "tunnelServo");


        // Reverse motors on left side

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);


        tunnelServo.setPosition(0);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        if (opModeIsActive()) {

            move(0, -1, 0, 1.5); // Move forwards?

            delay(2); // Wait




            telemetry.addData("Status", "Autonomous Complete");

            telemetry.update();

        }
    }
}
