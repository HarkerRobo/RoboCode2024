package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap.Pivot.Goal;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.drivetrain.AlignToStage;
//import frc.robot.commands.CommandGroups;
// import frc.robot.commands.drivetrain.AlignToStage;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.indexer.IndexToShooter;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.OuttakeNote;
import frc.robot.commands.intake.ZeroIntake;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.ZeroPivot;
import frc.robot.commands.shooter.MoveNoteToShooter;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.Flip;
import frc.robot.util.XboxGamepad;

public class OI {
    private static OI instance;

    private final Drivetrain m_drive = Drivetrain.getInstance();
    private final Pivot m_pivot = Pivot.getInstance();

    private XboxGamepad driver;
    private XboxGamepad operator;

    public OI() {
        driver = new XboxGamepad(RobotMap.OI.DRIVER_ID);
        operator = new XboxGamepad(RobotMap.OI.OPERATOR_ID);
        initBindings();
    }

    public XboxGamepad getDriver() {
        return driver;
    }

    public XboxGamepad getOperator() {
        return operator;
    }

    private void initBindings() {
        driver.getLeftBumper().onTrue(CommandGroups.getFullShootAmp());
        driver.getRightBumper().onTrue(CommandGroups.getFullShootSpeaker());

        driver.getUpDPadButton().onTrue(new PivotToAngle(Goal.SPEAKER));
        driver.getDownDPadButton().onTrue(new PivotToAngle(Goal.SUB));

        driver.getButtonB().whileTrue(new InstantCommand(() -> Pivot.getInstance().setPercentOutput(0.1)));
        driver.getButtonB().whileFalse(new InstantCommand(() -> Pivot.getInstance().setPercentOutput(0)));
        // driver.getButtonA().onTrue(new AlignToStage("left"));

        driver.getButtonY().onTrue(new ElevatorManual(RobotMap.Elevator.EXTEND_SPEED));
        driver.getButtonA().onTrue(new ElevatorManual(-RobotMap.Elevator.EXTEND_SPEED));

        driver.getButtonStart().onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().toggleRobotCentric();
        }));

        driver.getButtonX().onTrue(new InstantCommand( () -> Drivetrain.getInstance().setPose(new Pose2d(new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(121.25)), new Rotation2d(Math.toRadians(180))))));
        
        operator.getDownDPadButton().onTrue(CommandGroups.getFullZeroCommand());
        
        operator.getRightBumper().onTrue(CommandGroups.getFullIntakeCommand());
        operator.getLeftBumper().whileTrue(new OuttakeNote());


        //TESTING
        // operator.getUpDPadButton().onTrue(CommandGroups.PRE_ALIGN_CLIMB);
        // operator.getDownDPadButton().onTrue(CommandGroups.POST_ALIGN_CLIMB);
        // operator.getRightDPadButton().onTrue(CommandGroups.FULL_SHOOT_TRAP);

        // driver.a().whileTrue(m_pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driver.b().whileTrue(m_pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driver.x().whileTrue(m_pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driver.y().whileTrue(m_pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // driver.a().whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driver.b().whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driver.x().whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driver.y().whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
}