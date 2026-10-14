package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Matrix;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
                
    // 1. Mecanum drivetrain
    public static MecanumConfig drivetrainConfig =
        new MecanumConfig(c -> {
        
        c.frontLeftName.set("left_front");
        c.backLeftName.set("left_back");
        c.frontRightName.set("right_front");
        c.backRightName.set("right_back");
        
        // Example directions only.
        // Confirm these using the drivetrain tuner.
        c.frontLeftDirection.set(
        DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(
        DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(
        DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(
        DcMotorSimple.Direction.FORWARD);
        
        c.manualBrakeMode.set(true);
    });
    
    // 2. Two-wheel localization
    // Replace this with the complete configuration generated
    // by Pedro's Two Wheel Tuner.
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("left_back");
        c.yPodName.set("left_front");
        c.imuName.set("imu");
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.forwardTicksToInches.set(0.0031248);
        c.strafeTicksToInches.set(0.0031248);
        c.xPodDirection.set(Encoder.FORWARD);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
    });
    
    // 3. Path-following algorithm
    // These are starting defaults, NOT tuned robot values.
    public static ForesightConfig foresightConfig =
        new ForesightConfig(c -> {
            // Mandatory Variables for Foresight
            // Replace these with actual values from the Foresight Tuner.
            c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.1)));
            c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.1)));
            c.headingFeedback.set(Controller.proportional(1.0));
            
            c.coast.set(Controller.proportionalFeedforward(0.01));
            c.brake.set(Controller.proportionalFeedforward(0.01));

            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));
            c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
            
            c.maxAchievableForwardVelocity.set(60.0);
            c.maxAchievableStrafeVelocity.set(40.0);
            c.naturalForwardDeceleration.set(30.0);
            c.naturalStrafeDeceleration.set(30.0);

            // Tunable parameters
            c.headingDriveRatio.set(0.5);
            c.brakeAtEnd.set(true);
            
            c.translationalDeviationTolerance.set(2.5);
            c.headingDeviationTolerance.set(Math.toRadians(11.25));
            c.cosineScale.set(false);
            
            c.brakeAggression.set(1.0);
            c.maxBrakingPower.set(0.2);
            
            c.maxAccelerationConstraint.set(
            ForesightConfig.Constraint.NONE);
            c.maxVelocityConstraint.set(
            ForesightConfig.Constraint.NONE);
            c.maxDecelerationConstraint.set(
            ForesightConfig.Constraint.NONE);
            
            c.coastDownToVelocity.set(0.0);
        }
    );
    
    // 4. Create the follower
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new TwoWheelLocalizer(
                        hardwareMap, localizerConfig),
                new Mecanum(
                        hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
