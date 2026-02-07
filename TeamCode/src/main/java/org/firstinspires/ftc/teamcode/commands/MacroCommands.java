package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemNew;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@Config
public class MacroCommands {
    // TODO: totally wrong values and incorrect code
    public static long initialFlywheelSpinUp = 5000, kickDelay = 900, intakeFeedTime = 1000, spinUp = 250, intakeTime = 3000;

    public static Command launchSequence(IntakeSubsystemNew intake, KickerSubsystem kicker) {
        return new SequentialCommandGroup(


                new InstantCommand(() -> intake.setPower(12), intake),

                new WaitCommand(intakeFeedTime),
                new InstantCommand(() -> intake.setPower(0), intake),

                new WaitCommand(spinUp),



                new InstantCommand(() -> intake.setPower(12), intake),

                new WaitCommand(intakeFeedTime),

                new InstantCommand(() -> intake.setPower(0), intake),

                new WaitCommand(spinUp),

                new InstantCommand(kicker::moveToTarget),

                new WaitCommand(kickDelay),

                new InstantCommand(kicker::moveToHome)

        );
    }
}
