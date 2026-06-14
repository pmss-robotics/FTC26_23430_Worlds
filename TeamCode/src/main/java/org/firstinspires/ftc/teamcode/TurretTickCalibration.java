package org.firstinspires.ftc.teamcode; // Change this to match your folder structure

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Tick Calibration", group = "Test")
public class TurretTickCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Grab the turret motor exactly as it's named in your subsystem
        DcMotor turretMotor = hardwareMap.get(DcMotor.class, "turret");

        // Reset the encoder so your starting point is exactly 0
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set to FLOAT instead of BRAKE so you can easily spin it by hand
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Turret motor initialized and zeroed.");
        telemetry.addLine("Press Start, then move the turret by hand.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int currentTicks = turretMotor.getCurrentPosition();

            // Calculate what your CURRENT constant thinks the angle is
            double currentEstimate = (currentTicks / 548.0) * 360.0;

            telemetry.addLine("--- Turret Tick Test ---");
            telemetry.addData("Raw Ticks", currentTicks);
            telemetry.addData("Est. Degrees (using 548)", "%.2f", currentEstimate);
            telemetry.update();
        }
    }
}