package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.util.InternalPosition.mirrorIf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.MacroCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;
import com.skeletonarmy.marrow.settings.Settings;

import java.security.KeyStore;

@Autonomous(name = "15|RED|", group = "Autonomous")
@Configurable // Panels
public class red extends CommandOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class


    PedroDriveSubsystem drive;
    LoopTimer timer;
    private Prompter prompter;

    @Override
    public void initialize() {
        super.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        // Initialize subsystems here
        if (Settings.get("loop_detect_mode", false)) {
            timer = new LoopTimer(telemetry, "Main");
        }


        prompter = new Prompter(this);

        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", States.Alliance.Red, States.Alliance.Blue))
                .onComplete(() -> {
                    paths = new Paths(drive, prompter);
                    createSequence();
                });


        prompter.run();
    }

    public void createSequence() {


        Command sequence = new SequentialCommandGroup(
                new WaitUntilCommand(() -> !opModeInInit()), // Wait until start so you don't cry when robot moves in auto so its unusable yk
                new FollowPathCommand(drive.follower, paths.Path1)//, // Path1 whatever it is

                //MacroCommands.launchSequence()
        );
        schedule(sequence);
    }
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(PedroDriveSubsystem drive, Prompter prompter) {
            Follower follower = drive.follower;
            StateTransfer.alliance = prompter.get("alliance");
            boolean flip = States.Alliance.Red == StateTransfer.alliance;

            follower.setStartingPose(mirrorIf(116.3, 129.7, -142, flip));

            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(116.300, 129.700), flip),
                                    mirrorIf(new Pose(102.000, 103.000), flip),
                                    mirrorIf(new Pose(85.000, 72.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-142), Math.toRadians(0))
                    .setReversed()
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(85.000, 72.000), flip),
                                    mirrorIf(new Pose(97.000, 58.000), flip),
                                    mirrorIf(new Pose(132.000, 58.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(132.000, 58.000), flip),
                                    mirrorIf(new Pose(104.000, 57.000), flip),
                                    mirrorIf(new Pose(85.000, 72.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(85.000, 72.000), flip),
                                    mirrorIf(new Pose(105.000, 56.000), flip),
                                    mirrorIf(new Pose(132.000, 58.000), flip)
                            )

                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(132.000, 58.000), flip),
                                    mirrorIf(new Pose(110.000, 67.000), flip),
                                    mirrorIf(new Pose(94.000, 79.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(94.000, 79.000), flip),
                                    mirrorIf(new Pose(110.000, 85.000), flip),
                                    mirrorIf(new Pose(127.000, 84.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(127.000, 84.000), flip),
                                    mirrorIf(new Pose(106.000, 77.000), flip),
                                    mirrorIf(new Pose(85.000, 72.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(85.000, 72.000), flip),
                                    mirrorIf(new Pose(83.000, 30.000), flip),
                                    mirrorIf(new Pose(129.000, 36.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirrorIf(new Pose(129.000, 36.000), flip),
                                    mirrorIf(new Pose(100.000, 45.000), flip),
                                    mirrorIf(new Pose(85.000, 72.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirrorIf(new Pose(85.000, 72.000), flip),

                                    mirrorIf(new Pose(131.000, 55.000), flip)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(39))

                    .build();
        }
    }



}

