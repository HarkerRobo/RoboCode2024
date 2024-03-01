package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class RevShooter extends Command {

    private RobotMap.Shooter.Goal setpoint;

    public RevShooter(RobotMap.Shooter.Goal goal) {
        setpoint = goal;
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        switch (setpoint) {
            case AMP:
                Shooter.getInstance().setShooter(RobotMap.Shooter.INDEXING_SPEED);
                break;
            case SPEAKER:
                Shooter.getInstance().setShooter(RobotMap.Shooter.SHOOTING_SPEED);
                break;
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        Shooter.getInstance().setShooter(0);
        Shooter.getInstance().setIndexer(0);
    }
    
}
