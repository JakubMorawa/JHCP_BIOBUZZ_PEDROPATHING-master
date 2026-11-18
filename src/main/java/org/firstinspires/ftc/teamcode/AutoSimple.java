package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoSimple", group = "Robot")
public class AutoSimple extends LinearOpMode {
    // Control Hub Motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    // Expansion Hub Motors
    private DcMotor intakeMotor = null;
    private DcMotor elevatorMotor = null;
    private DcMotorEx shooterMotor = null; // Changed to DcMotorEx to match TeleOp

    // Servos
    private CRServo rightIntake = null;
    private CRServo leftIntake = null;

    TestBenchIMU bench = new TestBenchIMU();

    private ElapsedTime runtime = new ElapsedTime();

    // Constants from TeleOpBiobuzz
    private double elevatorPower = 0.75;
    private int targetVelocity = 1250;
    private double P = 130;
    private double I = 2;
    private double F = 17;

    // Constants for odometry
    final double COUNTS_PER_REV = 1900.0;
    final double WHEEL_DIAMETER_INCHES = 1.88976;
    final double INCHES_PER_COUNT = (Math.PI * WHEEL_DIAMETER_INCHES) / COUNTS_PER_REV;


    // ----------------- Helper Methods -----------------

    /**
     * Movement logic imported from TeleOpBiobuzz
     */
    private void move(double y, double x, double r) {
        double leftFrontPower = y + x - r;
        double leftBackPower = y - x - r;
        double rightFrontPower = y - x + r;
        double rightBackPower = y + x + r;

        double magnitude = 1.0;
        magnitude = Math.max(magnitude, Math.abs(leftFrontPower));
        magnitude = Math.max(magnitude, Math.abs(leftBackPower));
        magnitude = Math.max(magnitude, Math.abs(rightFrontPower));
        magnitude = Math.max(magnitude, Math.abs(rightBackPower));

        leftFront.setPower(leftFrontPower / magnitude);
        leftBack.setPower(leftBackPower / magnitude);
        rightFront.setPower(rightFrontPower / magnitude);
        rightBack.setPower(rightBackPower / magnitude);
    }

    /**
     * Move for a specific duration
     */
    private void move(double y, double x, double r, double duration) {
        move(y, x, r);
        delay(duration);
        stopDriveMotors();
    }

    private void stopDriveMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private void delay(double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            // Idle
        }
    }

    // Helper functions for robot mechanisms matching TeleOpBiobuzz logic
    private void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    private void setElevatorPower(double power) {
        elevatorMotor.setPower(power);
    }

    /**
     * Sets shooter velocity using PIDF values from TeleOp
     */
    private void setShooterVelocity(int velocity) {
        shooterMotor.setVelocity(velocity);
    }

    private void setIntakeServos(double power) {
        if (rightIntake != null && leftIntake != null) {
            rightIntake.setPower(-power);
            leftIntake.setPower(power);
        }
    }

    /**
     * Imported PIDF logic from TeleOpBiobuzz
     */
    public void setPIDFValues() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, 0.0, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }


    // ----------------- Main Autonomous Routine -----------------

    @Override
    public void runOpMode() {
        // Initialize hardware
        bench.init(hardwareMap);

        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        // Set up shooter as in TeleOp
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPIDFValues();

        // Set Directions to match TeleOpBiobuzz
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // START OF AUTO SEQUENCE

            // 1. Turn on shooter
            setShooterVelocity(targetVelocity);

            // 2. Turn on elevator
            setElevatorPower(elevatorPower);
            delay(2.0);

            // 3. Turn on intake servo motors
            setIntakeServos(1.0);

            // Wait for 3 seconds to allow mechanisms to run
            delay(3.0);

            // 4. Turn off all motors
            setShooterVelocity(0);
            setElevatorPower(0);
            intakeMotor.setPower(0);
            setIntakeServos(0);
            stopDriveMotors();

            // 5. Move forward (using positive Y for forward)
            move(0.5, 0, 0, 1.5);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}
