package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "BiobuzzAuto", group = "Robot")

public class BiobuzzAuto extends LinearOpMode {
    //control hub 
    private DcMotor leftFront = null; //0
    private DcMotor leftBack = null //1
    private DcMotor rightFront = null; //2
    private DcMotor rightBack = null; //3
    //expansion hub
    private DcMotor intakeMotor = null; //0
    private DcMotor shooterMotor = null; //1
    private DcMotor elevatorMotor = null; //2
    //control hub servo 
    private Servo filterServo = null; //0
    
    double shooterMotorPower = 0;
    double elevatorPower = 0;

    private final double FORWARD_SPEED = 1.0;

    private ElapsedTime runtime = new ElapsedTime();


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

        left_front.setPower(leftFrontPower / magnitude);
        left_back.setPower(leftBackPower / magnitude);
        right_front.setPower(rightFrontPower / magnitude);
        right_back.setPower(rightBackPower / magnitude);

    }


    private void moveAtAngle(double angleDegrees, double speed, double duration) {
        //the angle starting from the right side of robot's front
        double radians = Math.toRadians(angleDegrees);
        double y = Math.sin(radians); // forward component
        double x = Math.cos(radians);  // strafe component
        
        // call your existing move() function with calculated x/y

        move(x, y, 0, duration);

    }

    private void stopDriveMotors() {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
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

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");

        filterServo = hardwareMap.get(Servo.class, "filterServo");

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
            move(0, 1, 0, 1.5); // Move forward 1.5 seconds

            delay(2); // Wait




            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();

        }
    }
}
