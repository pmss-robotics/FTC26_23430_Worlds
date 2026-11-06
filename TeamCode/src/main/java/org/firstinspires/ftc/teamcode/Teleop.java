package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/**
 * Controls (modeled on last year's):
 *
 * DRIVER (gamepad1)
 *   Left stick          drive (field centric)
 *   Right stick X       turn
 *   Left bumper         intake while held, flywheel spins up when released
 *   Y                   intake while held (feeds balls into the flywheel) and keeps flywheel running
 *   B                   reverse intake while held (unjam)
 *   Right bumper        flywheel off
 *   Options             reset field-forward heading
 *
 * CO-DRIVER (gamepad2, optional, gamepad1 does everything else without it)
 *   Dpad up / down      flywheel rpm +/- 50
 */
@Config
@TeleOp(name = "Teleop", group = "TeleOp")
public class Teleop extends LinearOpMode {
    public static double flywheelRpm = 3000;
    public static double rpmStep = 50;

    private boolean wasLeftBumper, wasDpadU2, wasDpadD2;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        FlywheelSubsystem flywheel = new FlywheelSubsystem(hardwareMap);

        double rpmOffset = 0;
        boolean flywheelOn = false;

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.periodic();
            flywheel.periodic();

            // ---- Drive ----
            if (gamepad1.options) drive.resetHeading();
            drive.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // ---- Intake / flywheel ----
            if (gamepad1.left_bumper) {
                intake.on();
                flywheelOn = false;
            } else if (gamepad1.y) {
                intake.on();
                flywheelOn = true;
            } else if (gamepad1.b) {
                intake.reverse();
            } else {
                intake.off();
                // Releasing left bumper spins the flywheel up, like last year
                if (wasLeftBumper) flywheelOn = true;
            }
            wasLeftBumper = gamepad1.left_bumper;
            if (gamepad1.right_bumper) flywheelOn = false;

            // ---- RPM offset (co-driver) ----
            if (gamepad2.dpad_up && !wasDpadU2) rpmOffset += rpmStep;
            if (gamepad2.dpad_down && !wasDpadD2) rpmOffset -= rpmStep;
            wasDpadU2 = gamepad2.dpad_up;
            wasDpadD2 = gamepad2.dpad_down;

            flywheel.setVelocityRpm(flywheelOn ? flywheelRpm + rpmOffset : 0);

            telemetry.addData("heading (deg)", Math.toDegrees(drive.getHeading()));
            telemetry.addData("flywheel on", flywheelOn);
            telemetry.addData("target rpm", flywheel.getTargetRpm());
            telemetry.addData("actual rpm", flywheel.getActualRpm());
            telemetry.addData("at speed", flywheel.atSpeed());
            telemetry.addData("rpm offset", rpmOffset);
            telemetry.addData("intake power", intake.getPower());
            telemetry.update();
        }

        drive.stop();
        intake.off();
        flywheel.stop();
    }
}
