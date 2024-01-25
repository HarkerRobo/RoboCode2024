package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class ShooterManual extends Command {

    public ShooterManual() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        Shooter.getInstance().setShooter(1);
        try (Notifier waitForShooterRev = new Notifier(() -> {Shooter.getInstance().setIndexer(1); })) {
            waitForShooterRev.startSingle(RobotMap.Shooter.REV_TIME);
        }
        
    }

    public boolean isFinished() {
        return !Shooter.getInstance().shooterIndexerOccupied();
    }

    public void end() {
        Shooter.getInstance().setShooter(0);
    }
    
}
