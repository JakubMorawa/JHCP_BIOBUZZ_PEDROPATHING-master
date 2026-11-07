package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoShoot", group = "Robot")
public class AutoShoot extends LinearOpMode {
    // Control Hub Motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    // Expansion Hub Motors
    private DcMotor intakeMotor = null;
    private DcMotor elevatorMotor = null;
    private DcMotor shooterMotor = null;

    // Servos
    private CRServo rightIntake = null;
    private CRServo leftIntake = null;

    TestBenchIMU bench = new TestBenchIMU();

    private final double FORWARD_SPEED = 1.0;
    private ElapsedTime runtime = new ElapsedTime();

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

    private void moveAtAngle(double angleDegrees, double speed, double duration) {
        double radians = Math.toRadians(angleDegrees);
        double y = Math.sin(radians) * speed;
        double x = Math.cos(radians) * speed;
        move(y, x, 0, duration);
    }

    private void moveByOdometry(double targetStrafeInches, double targetForwardInches) {
        int startStrafeTicks = rightFront.getCurrentPosition();
        int targetStrafeTicks = (int) (targetStrafeInches / INCHES_PER_COUNT);

        int startForwardTicks = leftFront.getCurrentPosition();
        int targetForwardTicks = (int) (targetForwardInches / INCHES_PER_COUNT);

        // Strafe phase
        while (opModeIsActive() && Math.abs(rightFront.getCurrentPosition() - startStrafeTicks) < Math.abs(targetStrafeTicks)) {
            move(0, Math.signum(targetStrafeInches) * FORWARD_SPEED, 0);
            telemetry.addData("Strafe Ticks", rightFront.getCurrentPosition());
            telemetry.update();
        }

        // Forward phase
        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition() - startForwardTicks) < Math.abs(targetForwardTicks)) {
            move(Math.signum(targetForwardInches) * FORWARD_SPEED, 0, 0);
            telemetry.addData("Forward Ticks", leftFront.getCurrentPosition());
            telemetry.update();
        }
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

    // Helper functions for robot mechanisms
    private void intakeOn(double power) {
        intakeMotor.setPower(power);
    }

    private void elevatorOn(double power) {
        elevatorMotor.setPower(power);
    }

    private void shooterOn(double power) {
        shooterMotor.setPower(power);
    }

    private void setIntakeServos(double power) {
        if (rightIntake != null && leftIntake != null) {
            rightIntake.setPower(-power);
            leftIntake.setPower(power);
        }
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
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");



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

            // Example: Move forward for 1.5 seconds at half speed

            // Example: Turn on shooter and intake servos
            shooterOn(0.7);
            setIntakeServos(1.0);
            delay(2);
            elevatorOn(1);
            delay(5.0);


            // Stop everything
            elevatorOn(0);
            shooterOn(0);
            setIntakeServos(0);
            move(-0.6, 0, 0, 0.7);

            move(0, 0.6, 0, 3);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}
