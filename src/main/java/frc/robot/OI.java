package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CommandGroups;
//import frc.robot.commands.CommandGroups;
import frc.robot.commands.drivetrain.AlignToStage;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.indexer.IndexToShooter;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.ZeroIntake;
import frc.robot.commands.shooter.ShooterManual;
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
        // driver.getLeftBumper().onTrue(CommandGroups.FULL_SHOOT_AMP);
        // driver.getRightBumper().onTrue(CommandGroups.FULL_SHOOT_SPEAKER);
        driver.getButtonA().onTrue(new AlignToStage("center"));
        
        driver.getButtonSelect().onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().setYaw(0);
        }));

        driver.getButtonStart().onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().toggleRobotCentric();
        }));

        driver.getButtonX().onTrue(new InstantCommand( () -> Drivetrain.getInstance().setPose(
            Flip.apply(new Pose2d(new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(121.25)), new Rotation2d(0))))));
        driver.getButtonB().onTrue(CommandGroups.FULL_SHOOT_SPEAKER);
        operator.getDownDPadButton().onTrue(CommandGroups.FULL_ZERO);
        operator.getRightBumper().onTrue(CommandGroups.FULL_INTAKE);
        operator.getButtonY().whileTrue(new ElevatorManual());
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

        driver.getButtonY().whileTrue(new IntakeNote());
        driver.getButtonB().whileTrue(new IndexToShooter());
        // driver.getButtonB().whileTrue(new ZeroIntake());
        // driver.getButtonB().whileTrue(new ShooterManual(RobotMap.Shooter.Goal.SPEAKER));
    }

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
}