package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class ShootAmpNote extends Command {
    public ShootAmpNote() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        Shooter.getInstance().setIndexer(RobotMap.Shooter.SHOOTING_SPEED);
    }

    public boolean isFinished() {
        return !Shooter.getInstance().shooterIndexerOccupied();
    }

    public void end(boolean interrupted) {
        Shooter.getInstance().setShooter(0);
        Shooter.getInstance().setIndexer(0);
    }
}
