//100% DONE

package org.firstinspires.ftc.teamcode.pedroPathing;

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
import org.firstinspires.ftc.teamcode.util.StateTransfer;

@Autonomous(name = "Frog |R|15|", group = "Autonomous")
public class  FrogAutoRed extends OpMode {
    KickerSubsystem kicker;
    private Servo indicator;
    IntakeSubsystemNew intake;
    HoodSubsystem hood;
    FlywheelSubsystem outtake;
    TurretSubsystem turret;
    private boolean aimornot = false;
    public static double targetX = 144;
    public static double targetY = 144;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private Path Path1;
    private PathChain Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

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
        follower.setStartingPose(new Pose(114.22, 144 - 16.25, Math.toRadians(90)));
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

        StateTransfer.turretInitial = turret.getTurretAngle();

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
    }

    @Override
    public void stop() {
        StateTransfer.posePedro = follower.getPose();
    }

    public void buildPaths() {
        Path1 = new Path(new BezierLine(
                new Pose(114.22, 144 - 16.25),
                new Pose(92, 144 - 55.8)
        ));
        Path1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0));

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92, 144 - 55.8),
                                new Pose(83, 144 - 89),
                                new Pose(120, 144 - 89),
                                new Pose(106, 144 - 89),
                                new Pose(127, 144 - 77)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127, 144 - 77),
                                new Pose(98, 144 - 84),
                                new Pose(90, 144 - 66)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90, 144 - 66),
                                new Pose(98, 144 - 78),
                                new Pose(132.5, 144 - 83.67)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(21.5))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.5, 144 - 83.67),
                                new Pose(92, 144 - 80),
                                new Pose(90, 144 - 66)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(21.5), Math.toRadians(0))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90, 144 - 66),
                                new Pose(102, 144 - 60),
                                new Pose(124.000, 144 - 58)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 144 - 58),
                                new Pose(96.000, 144 - 62.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(96.000, 144 - 62.000),
                                new Pose(90.000, 144 - 110.000),
                                new Pose(130.000, 144 - 110.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130, 144 - 110.000),
                                new Pose(86, 144 - 34)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96, 144 - 62),
                                new Pose(102, 144 - 62)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.setPower(0);
                kicker.moveToTarget();
                aimornot = true;
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    aimornot = true;
                    follower.followPath(Path3);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 3.2) {
                    intake.setPower(0);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(1005);
                }
                break;
            case 1005:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path4, true);
                    setPathState(1006);
                }
                break;
            case 1006:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    intake.setPower(0);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(1007);
                }
                break;
            case 1007:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(2005);
                }
                break;
            case 2005:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path4, true);
                    setPathState(2006);
                }
                break;
            case 2006:
                if (pathTimer.getElapsedTimeSeconds() > 2.8) {
                    intake.setPower(0);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(2007);
                }
                break;
            case 2007:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path6);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    aimornot = true;
                    intake.setPower(0);
                    follower.followPath(Path7);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    intake.setPower(0);
                    follower.followPath(Path10);
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

    public static double computeY(double x) {
        return (0.0000175768 * Math.pow(x, 4)) - (0.00579237 * Math.pow(x, 3)) + (0.703251 * Math.pow(x, 2)) - (21.63918 * x) + 1940.14785;
    }

    public static double computeHoodPosition(double x) {
        return (-(1.67969 * Math.pow(10, -9)) * Math.pow(x, 4)) + ((5.93206 * Math.pow(10, -7)) * Math.pow(x, 3)) - 0.0000619875 * Math.pow(x, 2) + 0.00105249 * x + 0.38746;
    }
}