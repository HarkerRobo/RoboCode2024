package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import frc.robot.util.IndefiniteCommand;

public class AlignWithLimelight extends IndefiniteCommand {
    private ProfiledPIDController txController;
    private Timer minTime;

    public AlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());
        txController = new ProfiledPIDController(RobotMap.Limelight.LIMELIGHT_KP, RobotMap.Limelight.LIMELIGHT_KI, RobotMap.Limelight.LIMELIGHT_KD,  new Constraints(4, 4));
        txController.setIntegratorRange(-RobotMap.Limelight.LIMELIGHT_IZONE, RobotMap.Limelight.LIMELIGHT_IZONE);
        minTime = new Timer();

    }

    public void initialize() {
        minTime.reset();
        minTime.start();
        txController.reset(0);
    }

    public void execute() {
        double angularVelocity = -txController.calculate(Limelight.getTx(), 0);
        ChassisSpeeds chassis = new ChassisSpeeds(0, 0, -angularVelocity);
        Drivetrain.getInstance().setAngleAndDrive(chassis);
    }

    public boolean isFinished(){
        return minTime.hasElapsed(2) && Math.abs(Limelight.getTx()) < RobotMap.Limelight.LIMELIGHT_THRESHOLD;
    }

    public void end(boolean isFinished){
        ChassisSpeeds chassis = new ChassisSpeeds(0,0,0);
        Drivetrain.getInstance().setAngleAndDrive(chassis);
    }
}