package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.MathUtil;

public class PivotToAngle extends Command {
    private RobotMap.Pivot.Goal setpoint;
    private double ref;

    public PivotToAngle(RobotMap.Pivot.Goal goal) {
        setpoint = goal;
        addRequirements(Pivot.getInstance());
    }

    public void execute() {
        switch (setpoint) {
            case SPEAKER:
                ref = Pivot.getInstance().getPivotSetpoint(Drivetrain.getInstance().getDistanceToSpeaker());
                break;
            case AMP:
                ref = RobotMap.Pivot.AMP_ANGLE;
                break;
            case TRAP1:
                ref = RobotMap.Pivot.TRAP1_ANGLE;
                break;
            case TRAP2:
                ref = RobotMap.Pivot.TRAP2_ANGLE;
                break;
            case TRAP_SCORE:
                ref = RobotMap.Pivot.TRAP_SCORE_ANGLE;
                break;
        }
        Pivot.getInstance().moveToPosition(ref);

    }

    public boolean isFinished() {
        return MathUtil.compareSetpoint(Pivot.getInstance().getPosition(), ref, RobotMap.Pivot.MAX_ERROR);
    }

    // public void end(boolean interrupted) {
    //     Pivot.getInstance().setPercentOutput(0);
    // }

}
