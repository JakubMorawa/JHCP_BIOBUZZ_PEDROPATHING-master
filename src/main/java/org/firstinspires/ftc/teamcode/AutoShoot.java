package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoShoot", group = "Robot")

public class AutoShoot extends LinearOpMode {
    //control hub 
    private DcMotorEx leftFront = null; //0 //odom forward 1
    private DcMotorEx leftBack = null;//1  //odom forward 2
    private DcMotorEx rightFront = null; //2 //odom strafe
    private DcMotor rightBack = null; //3
    //expansion hub
    private DcMotor intakeMotor = null; //0
    private DcMotor shooterMotor = null; //1
    private DcMotor elevatorMotor = null; //2
    //control hub servo 
    private Servo filterServo = null; //0
    
    double shooterMotorPower = 0;
    double elevatorPower = 0;

    TestBenchIMU bench = new TestBenchIMU();

    private final double FORWARD_SPEED = 1.0;

    private ElapsedTime runtime = new ElapsedTime();

    final double COUNTS_PER_REV = 1900.0;
    final double WHEEL_DIAMETER_INCHES = 1.88976; // e.g., 48mm odom wheel = 1.889 inches
    final double INCHES_PER_COUNT = (Math.PI * WHEEL_DIAMETER_INCHES) / COUNTS_PER_REV;


    // ----------------- Helper Methods -----------------


    private void move(double x, double y, double r, double duration) {

        double leftFrontPower = (y - x + r)*FORWARD_SPEED;
        double leftBackPower = (y + x + r)*FORWARD_SPEED;
        double rightFrontPower = (y + x - r)*FORWARD_SPEED;
        double rightBackPower = (y - x - r)*FORWARD_SPEED;

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


    private void moveAtAngle(double angleDegrees, double duration) {
        //the angle starting from the right side of robot's front
        double radians = Math.toRadians(angleDegrees);
        double y = Math.sin(radians); // forward component
        double x = Math.cos(radians);  // strafe component
        
        // call your existing move() function with calculated x/y

        move(x, y, 0, duration);

    }

    private void moveByOdometry(double targetStrafeInches, double targetForwardInches) {
        // Reset or grab starting positions
        int startStrafeTicks = rightFront.getCurrentPosition();
        int targetStrafeTicks = (int) (targetStrafeInches / INCHES_PER_COUNT);

        int startForwardTicks = leftFront.getCurrentPosition();
        int targetForwardTicks = (int) (targetForwardInches / INCHES_PER_COUNT);
    
        while (opModeIsActive() && Math.abs(rightFront.getCurrentPosition() - startStrafeTicks) < Math.abs(targetStrafeTicks)) {
            // Apply drive power
            move(0, Math.signum(targetStrafeInches) * FORWARD_SPEED, 0, 0.02);
            
            // Print live odometry telemetry while moving
            telemetry.addData("Strafe Ticks", rightFront.getCurrentPosition());
            telemetry.addData("Foward Ticks", leftFront.getCurrentPosition());
            telemetry.update();
        }
        
        while (opModeIsActive() && Math.abs(rightFront.getCurrentPosition() - startForwardTicks) < Math.abs(targetForwardTicks)) {
            // Apply drive power
            move(Math.signum(targetForwardInches) * FORWARD_SPEED, 0, 0, 0.02);
            
            // Print live odometry telemetry while moving
            telemetry.addData("Strafe Ticks", rightFront.getCurrentPosition());
            telemetry.addData("Foward Ticks", leftFront.getCurrentPosition());
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


    private void setFilterPosition(double filterPosition, double waitDuration) {
        filterServo.setPosition(filterPosition);
        delay(waitDuration);
    }


    private void delay(double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime .seconds() < seconds) {
            // Keep looping without blocking
        }
    }

    private void intakeMotorOn(boolean state) {
        intakeMotor.setPower(state ? -1 : 0); 
        /*
            asks if the variable is true using the boolean and the quesiton mark
            if true, first number. if false, second number
        */
    }

    private void elevatorOn(boolean state) {
        elevatorMotor.setPower(state ? -1 : 0);
    }

    private void shooterMotorOn(boolean state, double power) {
        shooterMotor.setPower(state ? -1*power : 0);
    }


    // ----------------- Main Autonomous Routine -----------------


    @Override
    public void runOpMode() {

        // Initialize hardware
        bench.init(hardwareMap);
        
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");

        filterServo = hardwareMap.get(Servo.class, "filterServo");

        // Reset the encoder counts to zero at startup
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse motors on left side
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);


        filterServo.setPosition(0); //set the filter to the right position

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        if (opModeIsActive()) {
            // x: stafe, y:forward, r:rotation, duration:delay in seconds
            shooterMotorOn(true, 1);
            delay(1);
            elevatorOn(true);
            delay(1);
            shooterMotorOn(false, 0);
            elevatorOn(false);
            delay(1);

            moveAtAngle(45,0.5); // Move diagonally 0.5 seconds

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();

        }
    }
}
