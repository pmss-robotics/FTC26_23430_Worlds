package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous(name = "RED |15|", group = "Autonomous")
public class Red15 extends OpMode {
    KickerSubsystem kicker;
    private Servo indicator;
    IntakeSubsystemNew intake;
    HoodSubsystem hood;
    FlywheelSubsystem outtake;
    TurretSubsystem turret;
    private static boolean aimornot = false;
    public static double targetX = 71;
    public static double targetY = 71;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose pose1 = new Pose(115.956, 130.013, Math.toRadians(34));
    private final Pose pose2 = new Pose(84, 67, Math.toRadians(0));
    private final Pose pose3 = new Pose(126, 84, Math.toRadians(0));
    private final Pose pose4 = new Pose(85, 70, Math.toRadians(0));
    private final Pose pose5 = new Pose(100, 60, Math.toRadians(0));
    private final Pose pose6 = new Pose(124, 60, Math.toRadians(0));
    private final Pose pose7 = new Pose(85, 70, Math.toRadians(0));
    private final Pose pose8 = new Pose(85, 70, Math.toRadians(0));
    private final Pose pose9 = new Pose(132, 36, Math.toRadians(0));
    private final Pose pose10 = new Pose(85, 70, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2Start, grabPickup2End, scorePickup2, grabPickup3Start, grabPickup3End, scorePickup3;

    @Override
    public void init() {
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap, telemetry);
        outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        turret = new TurretSubsystem(hardwareMap);
        indicator = hardwareMap.get(Servo.class, "indicator");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(pose1);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        double distance = turret.getDistance();
        double hoodPos = computeHoodPosition(distance);

        hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
        hood.setHoodPosition(hoodPos);

        if (aimornot) {
            outtake.setVelocityRpm(computeY(turret.getDistance()));
        } else {
            outtake.setVelocityRpm(0);
        }

        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        double aimAngle = turret.calculateAimAngle(
                robotX, robotY, robotHeadingDeg,
                targetX, targetY
        );

        turret.goToAngle(aimAngle);

        double turretAngle = turret.getTurretAngle();
        double error = Math.abs(aimAngle - turretAngle);

        if (error < 2) {
            indicator.setPosition(0.611);
        } else {
            indicator.setPosition(0.277);
        }
    }

    @Override
    public void stop() {
        // No special stop actions needed
    }
    public void  buildPaths() {

            PathChain Path1;
            PathChain Path2;
            PathChain Path3;
            PathChain Path4;
            PathChain Path5;
            PathChain Path6;
            PathChain Path7;
            PathChain Path8;
            PathChain Path9;

                Path1 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(115.956, 130.013),
                                        new Pose(89.000, 97.000),
                                        new Pose(84.000, 67.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(34), Math.toRadians(0))

                        .build();

                Path2 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(84.000, 67.000),
                                        new Pose(110.000, 58.000),
                                        new Pose(120.000, 59.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path3 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(120.000, 59.000),
                                        new Pose(110.000, 58.000),
                                        new Pose(84.000, 67.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(84.000, 67.000),

                                        new Pose(130.000, 58.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(28))

                        .build();

                Path5 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(130.000, 58.000),
                                        new Pose(97.000, 63.000),
                                        new Pose(100.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(0))

                        .build();

                Path6 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(100.000, 84.000),

                                        new Pose(120.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path7 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(120.000, 84.000),

                                        new Pose(84.000, 67.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path8 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(84.000, 67.000),
                                        new Pose(97.000, 31.000),
                                        new Pose(120.000, 35.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path9 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(120.000, 35.000),

                                        new Pose(84.000, 67.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

        }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                new InstantCommand(() -> kicker.moveToTarget()).schedule();
                new InstantCommand(() -> aimornot = true).schedule();
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> kicker.moveToHome()).schedule();
                    new InstantCommand(() -> intake.setPower(0)).schedule();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    new InstantCommand(() -> kicker.moveToTarget()).schedule();
                    new InstantCommand(() -> aimornot = false).schedule();
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> aimornot = true).schedule();
                    follower.followPath(scorePickup1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> kicker.moveToHome()).schedule();
                    new InstantCommand(() -> intake.setPower(1)).schedule();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    new InstantCommand(() -> kicker.moveToTarget()).schedule();
                    new InstantCommand(() -> aimornot = false).schedule();
                    new InstantCommand(() -> intake.setPower(0)).schedule();
                    follower.followPath(grabPickup2Start);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> intake.setPower(1)).schedule();
                    follower.followPath(grabPickup2End, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> aimornot = true).schedule();
                    new InstantCommand(() -> intake.setPower(0)).schedule();
                    follower.followPath(scorePickup2);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> kicker.moveToHome()).schedule();
                    new InstantCommand(() -> intake.setPower(1)).schedule();
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    new InstantCommand(() -> kicker.moveToTarget()).schedule();
                    new InstantCommand(() -> aimornot = false).schedule();
                    new InstantCommand(() -> intake.setPower(0)).schedule();
                    follower.followPath(grabPickup3Start);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> intake.setPower(1)).schedule();
                    follower.followPath(grabPickup3End, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> aimornot = true).schedule();
                    new InstantCommand(() -> intake.setPower(0)).schedule();
                    follower.followPath(scorePickup3);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    new InstantCommand(() -> kicker.moveToHome()).schedule();
                    new InstantCommand(() -> intake.setPower(1)).schedule();
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    new InstantCommand(() -> kicker.moveToTarget()).schedule();
                    new InstantCommand(() -> aimornot = false).schedule();
                    new InstantCommand(() -> intake.setPower(0)).schedule();
                    setPathState(-1);
                }
                break;
            default:
                //Do nothing or end autonomous
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public static double computeY(double x) {
        return (-0.0000099416 * Math.pow(x, 4)) + (0.00464449 * Math.pow(x, 3)) - (0.79584 * Math.pow(x, 2)) + (73.70122 * x) - 132.82571;
    }

    // ⭐ HOOD QUADRATIC FORMULA
    public static double computeHoodPosition(double x) {
        return 1.04895 * Math.pow(10, -8) * Math.pow(x, 4) - 0.00000362859 * Math.pow(x, 3) + 0.000446154 * Math.pow(x, 2) - 0.0232164 * x + 0.77352;
    }

}