package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.swerve.Drivetrain;

public class PivotToAngle extends Command {
    public PivotToAngle() {
        addRequirements(Pivot.getInstance());
    }

    public void execute() {
        switch (Pivot.getInstance().getGoal()) {
            case SPEAKER:
                double dist = Drivetrain.getInstance().getDistanceToSpeaker();
                Pivot.getInstance().moveToPosition(Pivot.getInstance().getPivotSetpoint(dist));
                break;
            case AMP:
                Pivot.getInstance().moveToPosition(RobotMap.Pivot.AMP_ANGLE);
                break;
            case TRAP:
                Pivot.getInstance().moveToPosition(RobotMap.Pivot.TRAP_ANGLE);
                break;
            case INTAKE:
                Pivot.getInstance().moveToPosition(0);
                break;
        }

    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        Pivot.getInstance().moveToPosition(0);
        Pivot.getInstance().setPercentOutput(0);
    }

}
