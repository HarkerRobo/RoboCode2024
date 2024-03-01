package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
    public ShootNote() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        Shooter.getInstance().setIndexer(RobotMap.Shooter.SHOOTING_SPEED);
    }


    public void end(boolean interrupted) {
        Shooter.getInstance().setIndexer(0);
    }
}
