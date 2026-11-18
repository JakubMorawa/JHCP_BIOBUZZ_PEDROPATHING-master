package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * TeleOp Controls for BioBuzz:
 *
 * Gamepad 1 (Driver):
 * - Left Stick: Strafe (X) and Forward/Backward (Y)
 * - Right Stick: Rotation (X)
 * - Left Trigger (Hold): Precision Mode (40% Speed) AND Shooter Reverse
 * - Right Trigger (Hold): Shooter Forward (target velocity)
 * - Back Button: Toggle Field-Centric vs Robot-Centric
 * - Start Button: Reset IMU Heading (sets current front as "forward")
 * - D-Pad Up: Flip Robot Front (Robot-Centric mode only)
 * - D-Pad Left/Right: Adjust Shooter Velocity (-/+ 50)
 * - A Button: Intake Motor IN
 * - B Button: Elevator UP + Intake Motor IN
 * - Y Button: Elevator DOWN + Intake Motor OUT
 * - Right Bumper: Intake Servos IN
 * - Left Bumper: Intake Servos OUT
 *
 * Gamepad 2 (Tuning):
 * - D-Pad Right/Left: Adjust P (+/-)
 * - D-Pad Up/Down: Adjust F (+/-)
 * - X/B Button: Adjust I (-/+)
 * - Right Bumper: Cycle Increment (10, 1, 0.1, 0.01)
 * - A Button: Apply/Save PIDF values
 */
@TeleOp(name = "TeleOp BioBuzz", group = "Test")
public class TeleOpBiobuzz extends LinearOpMode {

    // Define encoder motor port mapped in configuration
    TestBenchIMU bench = new TestBenchIMU();
    private CRServo rightIntake = null;
    private CRServo leftIntake = null;
    
    // Control hub motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    
    // Expansion hub motors
    private DcMotor intakeMotor;
    private DcMotor elevatorMotor;
    private DcMotorEx shooterMotor;

    // --- Settings & Toggles ---
    private double elevatorPower = 0.75;
    private int targetVelocity = 1250;
    private int robotDirection = -1;
    
    // Default to Robot-Centric (false). Change to true if you prefer Field-Centric at start.
    private boolean isFieldCentric = false; 
    private double speedMultiplier = 1.0;  // For precision mode
    
    // --- PIDF Tuning ---
    private double P = 130, I = 2, F = 17;
    private final double[] changeList = {10, 1, 0.1, 0.01};
    private int changeListIndex = 0;

    @Override
    public void runOpMode() {
        // Initialize Hardware
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

        // Set drive motors to BRAKE for better control
        DcMotor[] driveMotors = {leftFront, leftBack, rightFront, rightBack};
        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPIDFValues();

        // Directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        teleopHeadingAndStatus();

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. DRIVE LOGIC ---
            
            // scaleInput cubes the stick values (x^3) for better fine control
            double lx = scaleInput(gamepad1.left_stick_x);
            double ly = scaleInput(gamepad1.left_stick_y);
            double rx = scaleInput(gamepad1.right_stick_x);

            // Toggle Field-Centric Mode (Back/Options Button)
            if (gamepad1.backWasPressed()) {
                isFieldCentric = !isFieldCentric;
            }

            // Reset IMU heading (Start Button)
            // Use this if your "Forward" direction drifts during a match
            if (gamepad1.startWasPressed()) {
                bench.init(hardwareMap); 
            }

            // Slow/Precision Mode (Hold Left Trigger for 40% speed)
            speedMultiplier = (gamepad1.left_trigger > 0.1) ? 0.4 : 1.0;

            if (isFieldCentric) {
                driveFieldCentric(ly, lx, rx);
            } else {
                // Robot-Centric uses the robotDirection toggle
                move(ly * robotDirection, lx * robotDirection, rx);
            }

            // Reverse Robot Front (Only affects Robot-Centric)
            if (gamepad1.dpadUpWasPressed()) {
                robotDirection *= -1;
            }

            // --- 2. MECHANISMS (Elevator & Intake) ---
            if (gamepad1.b) {
                elevatorMotor.setPower(elevatorPower);
                intakeMotor.setPower(1.0);
            } else if (gamepad1.y) {
                elevatorMotor.setPower(-elevatorPower);
                intakeMotor.setPower(-1.0);
            } else if (gamepad1.a) {
                intakeMotor.setPower(1.0);
                elevatorMotor.setPower(0);
            } else {
                elevatorMotor.setPower(0);
                intakeMotor.setPower(0);
            }

            // --- 3. SHOOTER (Triggers) ---
            if (gamepad1.right_trigger > 0.1) {
                shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                shooterMotor.setVelocity(targetVelocity);
            } else if (gamepad1.left_trigger > 0.1) {
                shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                shooterMotor.setVelocity(targetVelocity);
            } else {
                shooterMotor.setPower(0);
            }

            // Shooter target tuning
            if (gamepad1.dpadLeftWasPressed()) targetVelocity -= 50;
            if (gamepad1.dpadRightWasPressed()) targetVelocity += 50;

            // --- 4. INTAKE SERVOS (Bumpers) ---
            if (gamepad1.right_bumper) {
                rightIntake.setPower(-1);
                leftIntake.setPower(1);
            } else if (gamepad1.left_bumper) {
                rightIntake.setPower(1);
                leftIntake.setPower(-1);
            } else {
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }

            // --- 5. PIDF TUNING (Gamepad 2) ---
            handlePIDFTuning();

            // --- 6. TELEMETRY ---
            updateTelemetry(lx, ly, rx);
        }
    }

    /**
     * Field-Centric drive logic.
     * This "rotates" the joystick vector so that Forward is always away from the driver.
     */
    private void driveFieldCentric(double y, double x, double r) {
        // Get the current angle of the robot from the IMU
        double botHeading = Math.toRadians(-bench.getHeading());

        // Rotate the movement vector by the robot's heading
        // This is standard 2D rotation math
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        move(rotY, rotX, r);
    }

    private void move(double y, double x, double r) {
        double lf = (y + x - r) * speedMultiplier;
        double lb = (y - x - r) * speedMultiplier;
        double rf = (y - x + r) * speedMultiplier;
        double rb = (y + x + r) * speedMultiplier;

        // Ensure power levels stay between -1.0 and 1.0
        double max = Math.max(1.0, Math.abs(lf));
        max = Math.max(max, Math.abs(lb));
        max = Math.max(max, Math.abs(rf));
        max = Math.max(max, Math.abs(rb));

        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);
    }

    private double scaleInput(double input) {
        // Cubic scaling (input^3) gives much better precision at low stick values
        // while still maintaining 100% speed at the full stick push.
        if (Math.abs(input) < 0.05) return 0; // Small deadzone
        return input * input * input;
    }

    private void handlePIDFTuning() {
        double change = changeList[changeListIndex];
        if (gamepad2.dpadRightWasPressed()) P += change;
        if (gamepad2.dpadLeftWasPressed())  P -= change;
        if (gamepad2.dpadUpWasPressed())    F += change;
        if (gamepad2.dpadDownWasPressed())  F -= change;
        if (gamepad2.xWasPressed())         I -= change;
        if (gamepad2.bWasPressed())         I += change;
        
        if (gamepad2.rightBumperWasPressed()) {
            changeListIndex = (changeListIndex + 1) % changeList.length;
        }
        
        if (gamepad2.aWasPressed()) setPIDFValues();
    }

    private void updateTelemetry(double lx, double ly, double rx) {
        telemetry.addData("Mode", isFieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Speed Mode", speedMultiplier < 1.0 ? "PRECISION (40%)" : "FULL SPEED");
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("Vel (Target/Actual)", "%d / %.1f", targetVelocity, shooterMotor.getVelocity());
        telemetry.addData("PIDF", "P:%.1f I:%.1f F:%.1f", P, I, F);
        telemetry.addLine("--- Drive ---");
        telemetry.addData("Heading", "%.2f", bench.getHeading());
        telemetry.update();
    }

    private void teleopHeadingAndStatus() {
        telemetry.addData("Status", "Initialized. Robot-Centric Active.");
        telemetry.update();
    }

    public void setPIDFValues() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, 0.0, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}
