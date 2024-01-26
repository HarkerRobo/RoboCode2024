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
                Shooter.getInstance().setShooter(0.2);
                if (Limelight.atAmp()) {
                    Shooter.getInstance().setIndexer(0.2);
                }
                break;
            case SPEAKER:
                Shooter.getInstance().setShooter(1);
                if (Limelight.atSpeaker()) {
                    try (Notifier waitForShooterRev = new Notifier(() -> {Shooter.getInstance().setIndexer(1);})) {
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
