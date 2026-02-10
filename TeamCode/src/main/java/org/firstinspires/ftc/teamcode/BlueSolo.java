package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commands.PedroDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.StateTransfer;

import java.util.Objects;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BlueSolo", group = "TeleOp")
public class BlueSolo extends CommandOpMode {

    public static double driveSpeed = 1;
    public static double fast = 1;
    public static double slow = 1;

    public static double offset = 0;
    public static double targetX = 71;
    public static double targetY = 71;
    private static double shootvel = 0;
    private static boolean aimornot = false;

    private static int flywheelVelocity = 0;

    GamepadEx driver, tools;
    PedroDriveSubsystem drive;

    private Limelight3A limelight;
    private double tx = 0;
    private boolean hasTarget = false;

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
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);

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

        //new GamepadButton(driver, GamepadKeys.Button.B).toggleWhenPressed(
        //        new InstantCommand(() -> driveSpeed = slow),
        //        new InstantCommand(() -> driveSpeed = fast)
        //);

        new GamepadButton(driver, GamepadKeys.Button.B).toggleWhenPressed(
                new InstantCommand(() -> turret.holdAtZero(true)),
                new InstantCommand(() -> turret.holdAtZero(false))
        );

        // =========================
        // TOOLS CONTROLS
        // =========================

        // ⭐ NEW HOOD CONTROL — Button A uses quadratic formula
        //new GamepadButton(tools, GamepadKeys.Button.A).whenPressed(
        //        new InstantCommand(() -> {
        //            double distance = turret.getDistance();
        //            double hoodPos = computeHoodPosition(distance);

        //            hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
        //            hood.setHoodPosition(hoodPos);
        //        }, hood)
        //);

        // Reverse intake
        new GamepadButton(driver, GamepadKeys.Button.X).toggleWhenPressed(
                new InstantCommand(() -> intake.setPower(1.0)),
                new InstantCommand(() -> intake.setPower(0))
        );

        // Kicker toggle
        new GamepadButton(driver, GamepadKeys.Button.Y)
                .whileHeld(new RunCommand(() -> kicker.moveToHome()))
                .whenReleased(new InstantCommand(() -> kicker.moveToTarget()));


        // Belt feed
        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(() -> intake.setPower(1), intake))
                .whenReleased(new InstantCommand(() -> intake.setPower(0)));

        // Flywheel auto-RPM
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

        new GamepadButton(driver, GamepadKeys.Button.A)
                .whenPressed(new RunCommand(()->{
                    Pose currentPose = drive.getPose();
                    drive.setPose(new Pose(
                            currentPose.getX(),
                            currentPose.getY(),
                            0.0   // zero heading surely bwahah
                    ));
                }));



        // LIMELIGHT UPDATE
        schedule(new RunCommand(() -> {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tx = result.getTx();
                hasTarget = true;
            } else {
                tx = 0;
                hasTarget = false;
            }
        }));

        // MAIN LOOP
        schedule(new RunCommand(() -> {

            double distance = turret.getDistance();
            double hoodPos = computeHoodPosition(distance);

            hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
            hood.setHoodPosition(hoodPos);
            if (aimornot) {
                outtake.setVelocityRpm(computeY(turret.getDistance()));
            } else {
                outtake.setVelocityRpm(0);
            }
            Pose pose = drive.getPose();

            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());

            double aimAngle = turret.calculateAimAngle(
                    robotX, robotY, robotHeadingDeg,
                    targetX, targetY

            );

            turret.goToAngle(aimAngle + offset);

            double turretAngle = turret.getTurretAngle();
            double error = Math.abs(aimAngle - turretAngle);

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
            telemetry.addData("Distance to goal", turret.getDistance());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));

        schedule(driveCommand);
    }

    // Flywheel RPM curve
//    public static double computeY(double x) {
//        double exponent = -(0.0127895 * x - 0.9517);
//        double denominator = 1.0 + Math.exp(exponent);
//        return 5041.10161 / denominator;
//    }

    public static double computeY(double x) {
        return (0.0000175768 * Math.pow(x, 4)) - (0.00579237 * Math.pow(x, 3)) + (0.703251 * Math.pow(x, 2)) - (21.63918*x) + 1997.14785;
    }

    // ⭐ HOOD QUADRATIC FORMULA
//    public static double computeHoodPosition(double x) {
//        return 0.00000892857 * x * x
//                - 0.00195833 * x
//                + 0.43875;
//    }
    public static double computeHoodPosition(double x) {
        return (-(1.67969 * Math.pow(10, -9)) * Math.pow(x, 4)) + ((5.93206 * Math.pow(10, -7)) * Math.pow(x, 3)) - 0.0000619875 * Math.pow(x, 2) + 0.00105249*x + 0.38746;
    }
}