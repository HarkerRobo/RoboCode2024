package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.Limelight;

public class ShooterManual extends Command {

    private RobotMap.Shooter.Goal setpoint;
    private Notifier waitForShooterRev;
    private boolean revved;

    public ShooterManual(RobotMap.Shooter.Goal goal) {
        setpoint = goal;
        waitForShooterRev = new Notifier(() -> Shooter.getInstance().setIndexer(RobotMap.Shooter.SHOOTING_SPEED));
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        switch (setpoint) {
            case AMP:
                Shooter.getInstance().setShooter(RobotMap.Shooter.INDEXING_SPEED);
                if (Drivetrain.getInstance().alignedToAmp()) {
                    Shooter.getInstance().setIndexer(RobotMap.Shooter.INDEXING_SPEED);
                }
                break;
            case SPEAKER:
                Shooter.getInstance().setShooter(RobotMap.Shooter.SHOOTING_SPEED);
                if (Limelight.atSpeaker() && !revved) {
                    // Shooter.getInstance().setIndexer(RobotMap.Shooter.SHOOTING_SPEED);
                    waitForShooterRev.startSingle(RobotMap.Shooter.REV_TIME);
                    revved = true;
                }
                break;
        }
    }

    public boolean isFinished() {
        return !Shooter.getInstance().shooterIndexerOccupied();
    }

    public void end(boolean interrupted) {
        revved = false;
        Shooter.getInstance().setIndexer(0);
        // Shooter.getInstance().setShooter(0);
    }
    
}
