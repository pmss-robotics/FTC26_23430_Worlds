package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "Teleop", group = "Test")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drive.periodic();
            // Options resets field forward to the way the robot faces now
            if (gamepad1.options) drive.resetHeading();
            drive.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            // Hold X to run intake
            if (gamepad1.x) intake.on(); else intake.off();
            telemetry.addData("heading (deg)", Math.toDegrees(drive.getHeading()));
            telemetry.update();
        }
        drive.stop();
        intake.off();
    }
}
