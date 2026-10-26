package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Odometry Pod Test", group = "Test")
public class OdometryPodTest extends LinearOpMode {

    // Define encoder motor port mapped in configuration
    TestBenchIMU bench = new TestBenchIMU();
    private DcMotorEx left_front; // Will get bot position on the X-axis
    private DcMotorEx left_back;
    private DcMotorEx right_front;
    private DcMotor right_back;

    private DcMotor intakeMotor;

    // Constants for your specific odometry pod (update these!)
    // Example: REV Through Bore Encoder is 8192 counts per rev, or goBILDA/custom pods vary
    final double COUNTS_PER_REV = 1900.0;
    final double WHEEL_DIAMETER_INCHES = 1.88976; // e.g., 48mm odom wheel = 1.889 inches
    final double INCHES_PER_COUNT = (Math.PI * WHEEL_DIAMETER_INCHES) / COUNTS_PER_REV;

    @Override
    public void runOpMode() {
        // Initialize the hardware map
        bench.init(hardwareMap);
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Reset the encoder counts to zero at startup
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        teleopHeadingAndStatus(); // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;   // strafe
            double y = gamepad1.left_stick_y;  // forward
            double r = gamepad1.right_stick_x;    // rotation

            move(y, x, r);

            if (gamepad1.a){
                intakeMotor.setPower(-1);
            }
            else if (gamepad1.dpad_down){
                intakeMotor.setPower(.8);
            }
            else{
                intakeMotor.setPower(0.0);
            }

            // Read raw encoder ticks
            int rawTicksX = right_front.getCurrentPosition(); //front is strafe
            int rawTicksY = left_back.getCurrentPosition(); //front is strafe
            int rawTicksY2 = left_front.getCurrentPosition(); //back is forward

            // Calculate distance traveled in inches
            double distanceInchesX = rawTicksX * INCHES_PER_COUNT;
            double distanceInchesY = rawTicksY * INCHES_PER_COUNT;
            double distanceInchesY2 = rawTicksY2 * INCHES_PER_COUNT;

            telemetry.addData("Movement-", "");
            telemetry.addData("Strafe", x);
            telemetry.addData("Forward", y);
            telemetry.addData("Rotate", r);

            // Display data on Driver Hub screen
            telemetry.addData("Odometry Pods-", "");
            telemetry.addData("Raw Ticks X", rawTicksX);
            telemetry.addData("Raw Ticks Y", rawTicksY);
            telemetry.addData("Raw Ticks Y2", rawTicksY2);
            telemetry.addData("X-axis Distance (in)", "%.2f", distanceInchesX);
            telemetry.addData("Y-axis Distance (in)", "%.2f", distanceInchesY);
            telemetry.addData("Y2-axis Distance (in)", "%.2f", distanceInchesY2);

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

        double leftFrontPower = y + x + r;
        double leftBackPower = y - x + r;
        double rightFrontPower = y - x - r;
        double rightBackPower = y + x - r;

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
}
