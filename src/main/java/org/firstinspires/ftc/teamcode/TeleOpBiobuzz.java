package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name = "TeleOp BioBuzz", group = "Test")
public class TeleOpBiobuzz extends LinearOpMode {

   // Define encoder motor port mapped in configuration
    TestBenchIMU bench = new TestBenchIMU();
    private CRServo rightIntake = null;
    private CRServo leftIntake = null;
    //control hub
    private DcMotor leftFront; //0 forward1
    private DcMotor leftBack; //1 forward2
    private DcMotor rightFront; //2 strafe
    private DcMotor rightBack; //3
    //expansion hub
    private DcMotor intakeMotor; //0
    private DcMotor elevatorMotor; //1
    private DcMotor shooterMotor; //2
    
    //private Servo filterServo; //servo 0
    boolean isIntakeOn = false;

    private int robotDirection = -1;

    private double shooterPower = 0.7;
    private boolean prevStateB;
    private boolean prevStateRightBumper;

    // Constants for your specific odometry pod (update these!)
    final double COUNTS_PER_REV = 1900.0;
    final double WHEEL_DIAMETER_INCHES = 1.88976;
    final double INCHES_PER_COUNT = (Math.PI * WHEEL_DIAMETER_INCHES) / COUNTS_PER_REV;

    @Override
    public void runOpMode() {
        // Initialize the hardware map
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
        //filterServo = hardwareMap.get(Servo.class, "filterServo");

        // Reset the encoder counts to zero at startup
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        teleopHeadingAndStatus();

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            double x = robotDirection*gamepad1.left_stick_x; // strafe
            double y = robotDirection*gamepad1.left_stick_y; // forward
            double r = gamepad1.right_stick_x; // rotation

            move(y, x, r);

            if (gamepad1.b) {
                elevatorMotor.setPower(1);
            } else if (gamepad1.y) {
                elevatorMotor.setPower(-1);
            } else {
                elevatorMotor.setPower(0);
            }

            if (gamepad1.a || gamepad1.b) {
                intakeMotor.setPower(1);
            } else if (gamepad1.y) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }



            if (gamepad1.right_trigger > 0.1) {
                shooterMotor.setPower(0.7);
            } else if(gamepad1.left_trigger > 0.1) {
                shooterMotor.setPower(-0.7);
            }else if(gamepad1.dpad_left){
                shooterMotor.setPower(0.6);
            } else {
                shooterMotor.setPower(0);
            }

            if (gamepad1.dpadUpWasPressed()) {
                // Toggle the state
                robotDirection = -1*robotDirection;
            }
            if (gamepad1.right_bumper) {
                rightIntake.setPower(-1);
                leftIntake.setPower(1);
            } else if(gamepad1.left_bumper){
                rightIntake.setPower(1);
                leftIntake.setPower(-1);
            }else {
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }




            // Read raw encoder ticks
//            int rawTicksX = rightFront.getCurrentPosition(); // front is strafe
//            int rawTicksY = leftBack.getCurrentPosition(); // front is strafe
//            int rawTicksY2 = leftFront.getCurrentPosition(); // back is forward
//
//            // Calculate distance traveled in inches
//            double distanceInchesX = rawTicksX * INCHES_PER_COUNT;
//            double distanceInchesY = rawTicksY * INCHES_PER_COUNT;
//            double distanceInchesY2 = rawTicksY2 * INCHES_PER_COUNT;

            telemetry.addData("Movement-", "");
            telemetry.addData("Strafe", x);
            telemetry.addData("Forward", y);
            telemetry.addData("Rotate", r);

            // Display data on Driver Hub screen
//            telemetry.addData("Odometry Pods-", "");
//            telemetry.addData("Raw Ticks X", rawTicksX);
//            telemetry.addData("Raw Ticks Y", rawTicksY);
//            telemetry.addData("Raw Ticks Y2", rawTicksY2);
//            telemetry.addData("X-axis Distance (in)", "%.2f", distanceInchesX);
//            telemetry.addData("Y-axis Distance (in)", "%.2f", distanceInchesY);
//            telemetry.addData("Y2-axis Distance (in)", "%.2f", distanceInchesY2);

            telemetry.addData("IMU-", "");
            telemetry.addData("Heading", bench.getHeading());
            telemetry.addData("Pitch", bench.getPitch());
            telemetry.addData("Roll", bench.getRoll());

            telemetry.update();
        }
    }

    private void teleopHeadingAndStatus() {
        telemetry.addData("Status", "Initialized. Ready to start.");
        telemetry.update();
    }

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
}
