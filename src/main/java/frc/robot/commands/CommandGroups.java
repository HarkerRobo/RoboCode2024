package frc.robot.commands;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Pivot;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.indexer.IndexToShooter;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.ZeroIntake;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.QuickPivot;
import frc.robot.commands.pivot.ZeroPivot;
import frc.robot.commands.shooter.MoveNoteToShooter;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.shooter.ShootAmpNote;
import frc.robot.commands.shooter.ShootSpeakerNote;
import frc.robot.subsystems.Elevator;

public class CommandGroups {

        public static Command getFullZeroCommand() {
                return new ZeroPivot()
                .alongWith(new ZeroElevator(), new ZeroIntake());
        }

        public static Command getFullIntakeCommand() {
                return new MoveNoteToShooter()
                .raceWith(new IndexToShooter()
                .alongWith(new IntakeNote()))
                .andThen(new QuickPivot(), new ZeroPivot());
        }
    
        public static Command getFullShootSpeaker() {
                return new RevShooter(RobotMap.Shooter.Goal.SPEAKER)
                .alongWith(new PivotToAngle(RobotMap.Pivot.Goal.SPEAKER))
                .andThen(new ShootSpeakerNote())
                .andThen(new ZeroPivot());
        }
        
        public static Command getFullShootAmp() {
                return new PivotToAngle(RobotMap.Pivot.Goal.AMP)
                .andThen(new RevShooter(RobotMap.Shooter.Goal.AMP))
                .andThen(new ShootAmpNote()).andThen(new ZeroPivot());
        }

        public static Command getFullClimb() {
                return new InstantCommand(() -> Elevator.getInstance().setFollowerNeutralMode(
                        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)))
                .andThen(new PivotToAngle(Pivot.Goal.CLIMB)
                .alongWith(new MoveToPosition(RobotMap.Elevator.STAGE_HEIGHT)));
        }

        public static Command getFullRetractClimb() {
                return new MoveToPosition(0)
                .andThen(new InstantCommand(() -> Elevator.getInstance().setFollowerNeutralMode(
                        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))));
        }

        public static Command getFullShootNoAlign() {
                return new RevShooter(RobotMap.Shooter.Goal.AMP)
                .andThen(new ShootAmpNote()).andThen(new ZeroPivot());
        }
        
        
        
//        public static final Command PRE_DRIVEFWD_CLIMB = new PivotToAngle(RobotMap.Pivot.Goal.TRAP1); // wait for drive forward
//        public static final Command PRE_DRIVEBKWD_CLIMB = new PivotToAngle(RobotMap.Pivot.Goal.TRAP2).andThen(new MoveToPosition(RobotMap.Elevator.STAGE_HEIGHT).alongWith(new ZeroPivot())); // wait for drive backwards
//        public static final Command POST_DRIVEBKWD_CLIMB = new ZeroElevator();


//         public static final Command FULL_SHOOT_TRAP = new MoveToPosition(RobotMap.Elevator.TRAP_HEIGHT).alongWith(new PivotToAngle(RobotMap.Pivot.Goal.TRAP_SCORE))
                                                                                                //        .andThen(new ShooterManual(RobotMap.Shooter.Goal.AMP))
                                                                                                //        .andThen(new ZeroPivot(), new ZeroElevator());
    
}