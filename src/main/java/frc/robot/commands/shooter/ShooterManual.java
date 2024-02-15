package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class ShooterManual extends Command {

    private RobotMap.Shooter.Goal setpoint;

    public ShooterManual(RobotMap.Shooter.Goal goal) {
        setpoint = goal;
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        switch (setpoint) {
            case AMP:
                Shooter.getInstance().setShooter(RobotMap.Shooter.INDEXING_SPEED);
                if (Limelight.atAmp()) {
                    Shooter.getInstance().setIndexer(RobotMap.Shooter.INDEXING_SPEED);
                }
                break;
            case SPEAKER:
                Shooter.getInstance().setShooter(RobotMap.Shooter.SHOOTING_SPEED);
                if (Limelight.atSpeaker()) {
                    try (Notifier waitForShooterRev = new Notifier(() -> {Shooter.getInstance().setIndexer(RobotMap.Shooter.SHOOTING_SPEED);})) {
                        waitForShooterRev.startSingle(RobotMap.Shooter.REV_TIME);
                    } 
                }
                break;
        }
    }

    public boolean isFinished() {
        return !Shooter.getInstance().shooterIndexerOccupied();
    }

    public void end(boolean interrupted) {
        Shooter.getInstance().setShooter(0);
    }
    
}
