package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;

public class ShootSpeakerNote extends Command {
    public ShootSpeakerNote() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        if (Drivetrain.getInstance().alignedToSpeaker())
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
