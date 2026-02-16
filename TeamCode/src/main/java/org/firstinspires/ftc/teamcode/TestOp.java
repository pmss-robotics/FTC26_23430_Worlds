package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "TestOp", group = "Test")
public class TestOp extends CommandOpMode {

    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private TurretSubsystem turret;
    private IntakeSubsystemNew intake;
    private KickerSubsystem kicker;

    private GamepadEx tools;

    // ⭐ Pedro follower for live pose + distance updates
    private Follower follower;

    // Controller‑adjustable values
    private double testRPM = 0;
    private double testHoodPos = 0.33;

    private boolean flywheelOn = false;

    // Target for auto‑aim
    public static double targetX = 144;
    public static double targetY = 144;

    @Override
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = new FlywheelSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap, telemetry);
        turret = new TurretSubsystem(hardwareMap);
        intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);

        tools = new GamepadEx(gamepad2);

        // ⭐ Create Pedro follower so pose updates as robot moves
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));

        // ============================
        // CONTROLS
        // ============================

        // Toggle flywheel ON/OFF with A
        new com.seattlesolvers.solverslib.command.button.GamepadButton(
                tools, GamepadKeys.Button.A
        ).toggleWhenPressed(
                () -> flywheelOn = true,
                () -> flywheelOn = false
        );

        // Stop flywheel with B
        new com.seattlesolvers.solverslib.command.button.GamepadButton(
                tools, GamepadKeys.Button.B
        ).whenPressed(() -> {
            flywheelOn = false;
            flywheel.setVelocityRpm(0);
        });

        // Intake toggle (X)
        new com.seattlesolvers.solverslib.command.button.GamepadButton(
                tools, GamepadKeys.Button.X
        ).toggleWhenPressed(
                () -> intake.setPower(1.0),
                () -> intake.setPower(0)
        );

        // Kicker fire (Y)
        new com.seattlesolvers.solverslib.command.button.GamepadButton(
                tools, GamepadKeys.Button.Y
        ).whenPressed(() -> kicker.moveToHome())
                .whenReleased(() -> kicker.moveToTarget());

        // ============================
        // MAIN LOOP
        // ============================

        schedule(new RunCommand(() -> {

            tools.readButtons(); // ⭐ REQUIRED for debouncing

            // ⭐ Update pose from Pedro follower
            follower.update();
            Pose pose = follower.getPose();

            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());

            // ============================
            // DEBOUNCED RPM TUNING
            // ============================

            if (tools.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))   testRPM += 1000;
            if (tools.wasJustPressed(GamepadKeys.Button.DPAD_UP))     testRPM += 100;
            if (tools.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))  testRPM += 10;
            if (tools.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))   testRPM -= 100;

            if (testRPM < 0) testRPM = 0;

            // ============================
            // DEBOUNCED HOOD TUNING
            // ============================

            if (tools.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))  testHoodPos -= 0.005;
            if (tools.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) testHoodPos += 0.005;

            testHoodPos = Math.max(0.0, Math.min(1.0, testHoodPos));

            // ============================
            // APPLY VALUES
            // ============================

            if (flywheelOn) {
                flywheel.setVelocityRpm(testRPM);
            }

            hood.setHoodPosition(testHoodPos);

            // ============================
            // AUTO AIM (now uses REAL pose)
            // ============================

            double aimAngle = turret.calculateAimAngle(
                    robotX, robotY, robotHeadingDeg,
                    targetX, targetY
            );

            turret.goToAngle(aimAngle);

            // ============================
            // TELEMETRY
            // ============================

            telemetry.addLine("=== SHOOTER AIM TEST ===");
            telemetry.addData("Flywheel On", flywheelOn);
            telemetry.addData("Target RPM", testRPM);
            telemetry.addData("Actual RPM 1", flywheel.getActualRpm());
            telemetry.addData("Actual RPM 2", flywheel.getActualRpm2());
            telemetry.addData("Hood Position", testHoodPos);

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Heading", robotHeadingDeg);

            telemetry.addData("Turret Angle", turret.getTurretAngle());
            telemetry.addData("Aim Angle", aimAngle);
            telemetry.addData("Distance", turret.getDistance());

            telemetry.update();

        }));
    }
}
