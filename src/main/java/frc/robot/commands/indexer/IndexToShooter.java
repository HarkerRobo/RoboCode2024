package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Indexer;

public class IndexToShooter extends Command {

    public IndexToShooter() {
        addRequirements(Indexer.getInstance());
    }

    public void execute() {
        Indexer.getInstance().setPower(RobotMap.Indexer.INDEXING_SPEED);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        Indexer.getInstance().setPower(0);
    }
}
