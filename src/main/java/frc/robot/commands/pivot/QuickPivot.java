package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Pivot;

public class QuickPivot extends Command {
    public QuickPivot() {
        addRequirements(Pivot.getInstance());
    }

    public void execute() {
        Pivot.getInstance().setPercentOutput(0.3);
    }

    public boolean isFinished() {
        return Pivot.getInstance().getPosition() >= RobotMap.Pivot.QUICK_ANGLE;
    }

    public void end(boolean interrupted) {
        Pivot.getInstance().setPercentOutput(0);
    }
    
}
