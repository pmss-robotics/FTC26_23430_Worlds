package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretServoSubsystem;

@TeleOp(name = "Turret Servo Test", group = "Test")
public class TurretServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        TurretServoSubsystem turret = new TurretServoSubsystem(hardwareMap);
        telemetry.addLine("Ready. Right stick X spins turret, A recenters.");
        telemetry.update();
        waitForStart();
        turret.resetTimer();
        while (opModeIsActive()) {
            turret.drive(gamepad1.right_stick_x);
            if (gamepad1.a) turret.setPosition(0.5);
            telemetry.addData("turret position", "%.3f", turret.getPosition());
            telemetry.update();
        }
    }
}
