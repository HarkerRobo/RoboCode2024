package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
    public ShootNote() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        if (Shooter.getInstance().isShooterRevved())
        {
            Shooter.getInstance().setIndexer(RobotMap.Shooter.SHOOTING_SPEED);
        }
    }

    public boolean isFinished() {
        return false;
    }


    public void end(boolean interrupted) {
        Shooter.getInstance().setIndexer(0);
    }
}
