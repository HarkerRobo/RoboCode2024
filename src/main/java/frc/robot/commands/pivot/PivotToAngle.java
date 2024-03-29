package frc.robot.commands.pivot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.MathUtil;

public class PivotToAngle extends Command {
    private RobotMap.Pivot.Goal setpoint;
    private double ref;
    private Debouncer debouncer;

    public PivotToAngle(RobotMap.Pivot.Goal goal) {
        setpoint = goal;
        debouncer = new Debouncer(0.35, DebounceType.kRising); // 0.5 if not working
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
        double error;
        switch(setpoint) {
            case SPEAKER:
                error = RobotMap.Pivot.MAX_ERROR_SPEAKER;
                break;
            default:
                error = RobotMap.Pivot.MAX_ERROR_AMP;
        }
        return debouncer.calculate(MathUtil.compareSetpoint(Pivot.getInstance().getPosition(), ref, error));
    }

    public void end(boolean interrupted) {
        Pivot.getInstance().moveToPosition(ref);
    }

}
