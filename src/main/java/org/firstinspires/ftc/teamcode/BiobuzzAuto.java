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

    private Servo tunnelServo = null;

    double shooterMotorPower = 0;

    double elevatorPower = 0;



    private final double FORWARD_SPEED = 0.5;

    private ElapsedTime runtime = new ElapsedTime();


    // ----------------- Helper Methods -----------------


    private void move(double x, double y, double rx, double duration) {

        double left_frontPower = -(y - x - rx) * FORWARD_SPEED;

        double left_backPower = -(y + x - rx) * FORWARD_SPEED;

        double right_frontPower = -(y + x + rx) * FORWARD_SPEED;

        double right_backPower = -(y - x + rx) * FORWARD_SPEED;


        left_front.setPower(left_frontPower);

        right_front.setPower(right_frontPower);

        left_back.setPower(left_backPower);

        right_back.setPower(right_backPower);


        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < duration) {

            // Wait for move to complete

        }


        stopDriveMotors();

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