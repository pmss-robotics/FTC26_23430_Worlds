package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.PedroDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.StateTransfer;

import java.util.Objects;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SOTM", group = "TeleOp")
public class SOTM extends CommandOpMode {

    public static double driveSpeed = 1;
    public static double fast = 1;
    public static double slow = 1;

    public static double offset = 0;
    public static double targetX = 143;
    public static double targetY = 143;

    private static boolean aimornot = false;

    GamepadEx driver, tools;
    PedroDriveSubsystem drive;

    private Limelight3A limelight;

    private Servo indicator;

    @Override
    public void initialize() {

        if (Objects.isNull(StateTransfer.posePedro)) {
            StateTransfer.posePedro = new Pose(43.956, 58.013, Math.toRadians(-142.35));
        }

        FlywheelSubsystem outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        IntakeSubsystemNew intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap);
        HoodSubsystem hood = new HoodSubsystem(hardwareMap, telemetry);
        MovingWhileShooting turret = new MovingWhileShooting(hardwareMap);

        turret.setInitialAngle(StateTransfer.turretInitial);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driver = new GamepadEx(gamepad1);
        tools = new GamepadEx(gamepad2);

        drive = new PedroDriveSubsystem(
                Constants.createFollower(hardwareMap),
                StateTransfer.posePedro,
                telemetry
        );

        hood.moveToHome();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        indicator = hardwareMap.get(Servo.class, "indicator");
        indicator.setPosition(1.0);

        driveSpeed = fast;

        PedroDriveCommand driveCommand = new PedroDriveCommand(
                drive,
                () -> -driver.getLeftX() * driveSpeed,
                () -> driver.getLeftY() * driveSpeed,
                () -> -driver.getRightX() * driveSpeed,
                true
        );

        // =========================
        // DRIVER CONTROLS
        // =========================

        new GamepadButton(driver, GamepadKeys.Button.B).toggleWhenPressed(
                new InstantCommand(() -> turret.holdAtZero(true)),
                new InstantCommand(() -> turret.holdAtZero(false))
        );

        new GamepadButton(driver, GamepadKeys.Button.X).toggleWhenPressed(
                new InstantCommand(() -> intake.setPower(1.0)),
                new InstantCommand(() -> intake.setPower(0))
        );

        new GamepadButton(driver, GamepadKeys.Button.Y)
                .whileHeld(new RunCommand(kicker::moveToHome))
                .whenReleased(new InstantCommand(kicker::moveToTarget));

        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(() -> intake.setPower(1), intake))
                .whenReleased(new InstantCommand(() -> intake.setPower(0)));

        new GamepadButton(driver, GamepadKeys.Button.DPAD_UP).toggleWhenPressed(
                new InstantCommand(() -> aimornot = true),
                new InstantCommand(() -> aimornot = false)
        );

        new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> offset = offset + 4)
        );
        new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> offset = offset - 4)
        );
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> offset = 0)
        );
        new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> drive.setPose(new Pose(9, 9, 0)))
        );

        // MAIN LOOP
        schedule(new RunCommand(() -> {

            Pose pose = drive.getPose();
            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());

            // You may need to adjust this depending on PedroDriveSubsystem API
            Vector vel = drive.getVelocity(); // field-centric velocity
            double robotVx = vel.getXComponent();
            double robotVy = vel.getYComponent();

            // Lead-compensated aim
            double aimAngle = turret.calculateAimAngleWithLead(
                    robotX, robotY, robotHeadingDeg,
                    targetX, targetY,
                    robotVx, robotVy

            );

            turret.goToAngle(aimAngle + offset);

            // Effective distance after lead
            double distance = turret.getDistance();

            // Hood position from distance
            double hoodPos = computeHoodPosition(distance);
            hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
            hood.setHoodPosition(hoodPos);

            // Flywheel RPM from distance
            if (aimornot) {
                outtake.setVelocityRpm(computeY(distance));
            } else {
                outtake.setVelocityRpm(0);
            }

            double turretAngle = turret.getTurretAngle();
            double error = Math.abs(aimAngle - turretAngle + offset);

            if (error < 2) {
                indicator.setPosition(0.611);
            } else {
                indicator.setPosition(0.277);
            }

            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Aim Angle", aimAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Flywheel RPM", outtake.getTargetRPM());
            telemetry.addData("Actual Outtake RPM", outtake.getActualRpm());
            telemetry.addData("Actual Outtake 2 RPM", outtake.getActualRpm2());
            telemetry.addData("Distance to goal (effective)", distance);
            telemetry.addData("Robot Vx", robotVx);
            telemetry.addData("Robot Vy", robotVy);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));

        schedule(driveCommand);
    }

    public static double computeY(double x) {
        return (0.0000175768 * Math.pow(x, 4))
                - (0.00579237 * Math.pow(x, 3))
                + (0.703251 * Math.pow(x, 2))
                - (21.63918 * x)
                + 1997.14785;
    }

    public static double computeHoodPosition(double x) {
        return (-(1.67969e-9) * Math.pow(x, 4))
                + ((5.93206e-7) * Math.pow(x, 3))
                - (0.0000619875 * Math.pow(x, 2))
                + 0.00105249 * x
                + 0.38746;
    }
}
