package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.indexer.IndexToShooter;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.ZeroIntake;
import frc.robot.commands.pivot.ZeroPivot;
import frc.robot.commands.shooter.MoveNoteToShooter;
import frc.robot.commands.shooter.ShooterManual;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class CommandGroups {
        public static final Command FULL_ZERO = new ZeroPivot().alongWith(new ZeroElevator(), new ZeroIntake());
    
        public static final Command FULL_INTAKE = new MoveNoteToShooter().raceWith(new IndexToShooter().alongWith(new IntakeNote()));
    
        public static final Command FULL_SHOOT_SPEAKER = new InstantCommand(() -> {Pivot.getInstance().setGoal(RobotMap.Pivot.Goal.SPEAKER);}).andThen(new ShooterManual()).andThen(new InstantCommand(() -> {Pivot.getInstance().setGoal(RobotMap.Pivot.Goal.INTAKE);}));
    
        public static final Command FULL_SHOOT_AMP = new InstantCommand(() -> {Pivot.getInstance().setGoal(RobotMap.Pivot.Goal.AMP);}).andThen(new ShooterManual()).andThen(new InstantCommand(() -> {Pivot.getInstance().setGoal(RobotMap.Pivot.Goal.INTAKE);}));

        public static final Command PRE_ALIGN_CLIMB = new InstantCommand(() -> {Elevator.getInstance().setState(RobotMap.Elevator.State.STAGE);});

        public static final Command POST_ALIGN_CLIMB = new InstantCommand(() -> {Elevator.getInstance().setState(RobotMap.Elevator.State.IDLE);});

        public static final Command FULL_SHOOT_TRAP = new InstantCommand(() -> {Elevator.getInstance().setState(RobotMap.Elevator.State.TRAP); Pivot.getInstance().setGoal(RobotMap.Pivot.Goal.TRAP);}).andThen(new ShooterManual()).andThen(new InstantCommand(() -> {Pivot.getInstance().setGoal(RobotMap.Pivot.Goal.INTAKE);}));
    }