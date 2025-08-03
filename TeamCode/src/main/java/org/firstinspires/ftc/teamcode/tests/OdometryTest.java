package org.firstinspires.ftc.teamcode.tests;

// Import necessary classes
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "OdometryTest")
public class OdometryTest extends LinearOpMode {

    // Declare motors for odometry wheels
    DcMotor odometryLeft, odometryRight;

    // Parameters
    double wheelCircumference = Math.PI * 4.0; // In inches, example for a 4-inch diameter wheel
    double ticksPerRev = 1440; // Example: ticks per revolution for the GoBilda encoders (adjust as needed)

    // Variables to store encoder values
    double lastLeftTicks = 0;
    double lastRightTicks = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        odometryLeft = hardwareMap.dcMotor.get("odometry_left");  // Change to the actual motor name
        odometryRight = hardwareMap.dcMotor.get("odometry_right");  // Change to the actual motor name

        // Reset encoders
        odometryLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometryRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to run using encoders
        odometryLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the start command
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Get the current encoder values for the odometry wheels
            double leftTicks = odometryLeft.getCurrentPosition();
            double rightTicks = odometryRight.getCurrentPosition();

            // Calculate distance traveled for each wheel
            double leftDistance = (leftTicks - lastLeftTicks) * (wheelCircumference / ticksPerRev);
            double rightDistance = (rightTicks - lastRightTicks) * (wheelCircumference / ticksPerRev);

            // Display the distance traveled on telemetry
            telemetry.addData("Left Distance (inches)", leftDistance);
            telemetry.addData("Right Distance (inches)", rightDistance);
            telemetry.update();

            // Update the last encoder ticks for the next iteration
            lastLeftTicks = leftTicks;
            lastRightTicks = rightTicks;

            // Optional: Delay to make telemetry more readable
            sleep(100); // Adjust the delay as needed
        }
    }
}



