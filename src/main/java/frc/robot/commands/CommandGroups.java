package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.indexer.IndexToShooter;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.ZeroIntake;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.ZeroPivot;
import frc.robot.commands.shooter.MoveNoteToShooter;
import frc.robot.commands.shooter.RevShooter;

public class CommandGroups {
        public static final Command FULL_ZERO = new ZeroPivot().alongWith(new ZeroElevator(), new ZeroIntake());
    
        public static final Command FULL_INTAKE = new MoveNoteToShooter().alongWith(new ZeroPivot()).raceWith(new IndexToShooter().alongWith(new IntakeNote()));
    
        public static final Command PRE_SHOOT_SPEAKER = new PivotToAngle(RobotMap.Pivot.Goal.SPEAKER).alongWith(new RevShooter(RobotMap.Shooter.Goal.SPEAKER));
    
        public static final Command PRE_SHOOT_AMP = new PivotToAngle(RobotMap.Pivot.Goal.AMP).alongWith(new RevShooter(RobotMap.Shooter.Goal.AMP));
        
        
//        public static final Command PRE_DRIVEFWD_CLIMB = new PivotToAngle(RobotMap.Pivot.Goal.TRAP1); // wait for drive forward
//        public static final Command PRE_DRIVEBKWD_CLIMB = new PivotToAngle(RobotMap.Pivot.Goal.TRAP2).andThen(new MoveToPosition(RobotMap.Elevator.STAGE_HEIGHT).alongWith(new ZeroPivot())); // wait for drive backwards
//        public static final Command POST_DRIVEBKWD_CLIMB = new ZeroElevator();


//         public static final Command FULL_SHOOT_TRAP = new MoveToPosition(RobotMap.Elevator.TRAP_HEIGHT).alongWith(new PivotToAngle(RobotMap.Pivot.Goal.TRAP_SCORE))
                                                                                                //        .andThen(new ShooterManual(RobotMap.Shooter.Goal.AMP))
                                                                                                //        .andThen(new ZeroPivot(), new ZeroElevator());
    
}