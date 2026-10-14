package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Pedro Pathing Test", group = "Test")
public class pedropathingTest extends LinearOpMode {
    private Follower follower;

    @Override
    public void runOpMode() {
        // Initialize the follower using your Constants configuration
        follower = Constants.create(hardwareMap);

        // Define a simple path: move 24 inches forward
        // We use a constant heading of 0 degrees
        Path path = line(new Pose(0, 0, 0), new Pose(24, 0, 0)).constant(0);

        // Set the starting pose of the robot to (0,0,0)
        follower.setPose(new Pose(0, 0, 0));

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Start following the path
        follower.follow(path);

        while (opModeIsActive() && !isStopRequested()) {
            // Update the follower in every loop iteration
            follower.update();

            // Display the robot's current position via telemetry
            Pose currentPose = follower.pose();
            telemetry.addData("x", currentPose.x());
            telemetry.addData("y", currentPose.y());
            telemetry.addData("heading", Math.toDegrees(currentPose.heading()));
            telemetry.addData("isBusy", follower.isBusy());
            telemetry.update();
        }
    }
}
