package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.StateTransfer;

@Autonomous(name = "|R|18|FAR|", group = "Autonomous")
public class FarRedAuto18 extends OpMode {
    KickerSubsystem kicker;
    private Servo indicator;
    IntakeSubsystemNew intake;
    HoodSubsystem hood;
    FlywheelSubsystem outtake;
    MovingWhileShooting turret;
    private boolean aimornot = false;
    public static double targetX = 144;
    public static double targetY = 144;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;
    private int loopCount = 0;

    private Path Path1;
    private PathChain Path2, Path3, Path4, Path5, Path6, Path7, Path8;

    @Override
    public void init() {
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap, telemetry);
        outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        turret = new MovingWhileShooting(hardwareMap);
        indicator = hardwareMap.get(Servo.class, "indicator");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        follower.setStartingPose(new Pose(81.000, 9.000, Math.toRadians(0)));
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        hood.setHoodPosition(0.29);

        if (aimornot) {
            outtake.setVelocityRpm(3220);
        } else {
            outtake.setVelocityRpm(0);
        }

        Pose   pose            = follower.getPose();
        double robotX          = pose.getX();
        double robotY          = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        Vector vel     = follower.getVelocity();
        double robotVx = vel.getXComponent();
        double robotVy = vel.getYComponent();

        double aimAngle = turret.calculateAimAngleWithLead(
                robotX, robotY, robotHeadingDeg, targetX, targetY, robotVx, robotVy);
        turret.goToAngle(aimAngle);

        StateTransfer.turretInitial = turret.getTurretAngle();
        indicator.setPosition(Math.abs(aimAngle - turret.getTurretAngle()) < 2 ? 0.611 : 0.277);

        telemetry.addData("path state", pathState);
        telemetry.addData("loop count", loopCount);
        telemetry.update();
    }

    @Override
    public void stop() {
        StateTransfer.posePedro = follower.getPose();
    }

    public void buildPaths() {
        // (86,8) → (83,25): drive from start to shoot spot
        Path1 = new Path(new BezierLine(
                new Pose(81.000, 9.000),
                new Pose(82.000, 28.000)));
        Path1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20));

        // (83,25) → (133,9): shoot spot to sample 1
        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(82.000, 28.000),
                        new Pose(133.000, 9.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();

        // (133,9) → (83,25): sample 1 back to shoot spot
        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(133.000, 9.000),
                        new Pose(82.000, 28.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        // (83,25) → (122,35): shoot spot to sample 2 (curve)
        Path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(82.000, 28.000),
                        new Pose(82.000, 37.000),
                        new Pose(122.000, 35.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();

        // (122,35) → (83,25): sample 2 back to shoot spot
        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122.000, 35.000),
                        new Pose(82.000, 28.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        // (83,25) → (133,9): shoot spot to sample 3 — loops 3x
        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(82.000, 28.000),
                        new Pose(133.000, 9.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();

        // (133,9) → (83,25): sample 3 back to shoot spot — loops 3x
        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(133.000, 9.000),
                        new Pose(82.000, 28.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        // (83,25) → (90,25): park
        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(82.000, 28.000),
                        new Pose(91.000, 25.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── Drive from start (86,8) to shoot spot (83,25) ────────────
            case 0:
                intake.setPower(0);
                kicker.moveToTarget();
                aimornot = true;
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path2, true);
                    setPathState(4);
                }
                break;

            // ── Sample 1 (133,9) → return to shoot spot ───────────────────
            case 4:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path3);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path4, true);
                    setPathState(8);
                }
                break;

            // ── Sample 2 (122,35) → return to shoot spot ──────────────────
            case 8:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path6, true);
                    setPathState(12);
                }
                break;

            // ── LOOP: (83,25) → (133,9) and back, 3 total cycles ─────────
            case 12:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path7);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;

            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(15);
                }
                break;

            // ── Loop gate: 3 total cycles of Path6+Path7, then park ───────
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    loopCount++;
                    if (loopCount < 3) {
                        kicker.moveToTarget();
                        aimornot = false;
                        follower.followPath(Path6, true);
                        setPathState(12);
                    } else {
                        intake.setPower(0);
                        aimornot = false;
                        follower.followPath(Path8);
                        setPathState(16);
                    }
                }
                break;

            // ── Park (90,25) ───────────────────────────────────────────────
            case 16:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
