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
                Pivot.getInstance().moveToPosition(ref);
                break;
            case AMP:
                ref = RobotMap.Pivot.AMP_ANGLE;
                Pivot.getInstance().moveToPositionAmp(ref);
                break;
            case CLIMB:
                ref = RobotMap.Pivot.CLIMB_ANGLE;
                Pivot.getInstance().moveToPosition(ref);
                break;
        }

    }

    public double getRef() {
        return ref;
    }

    public boolean isFinished() {
        switch(setpoint) {
            case SPEAKER:
                return MathUtil.compareSetpoint(Pivot.getInstance().getPosition(), ref, RobotMap.Pivot.MAX_ERROR);
            default:
                return MathUtil.compareSetpoint(Pivot.getInstance().getPosition(), ref, RobotMap.Pivot.MAX_ERROR + 1);
        }
    }

    public void end(boolean interrupted) {
        Pivot.getInstance().moveToPosition(ref);
    }

}
