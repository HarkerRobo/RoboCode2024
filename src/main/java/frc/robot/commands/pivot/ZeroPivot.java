package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Pivot;

public class ZeroPivot extends Command {
    public ZeroPivot() {
        addRequirements(Pivot.getInstance());
    }

    public void execute() {
        Pivot.getInstance().setPercentOutput(RobotMap.Pivot.ZERO_SPEED);
        // Pivot.getInstance().setPercentOutput(0);
    }

    public boolean isFinished() {
        return Pivot.getInstance().getPosition() <= 1;
        // return Pivot.getInstance().isLimitHit(); 
        // return Pivot.getInstance().isStalling();
    }

    public void end(boolean interrupted) {
        Pivot.getInstance().setPercentOutput(0);
        // Pivot.getInstance().setSensorPosition(0);
    }
    
}