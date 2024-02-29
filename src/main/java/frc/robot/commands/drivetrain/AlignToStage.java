package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.Limelight;

public class AlignToStage extends Command {

    private static PIDController vxStageController = new PIDController(RobotMap.Drivetrain.VX_STAGE_kP, 0, 0);
    private static PIDController vyStageController = new PIDController(RobotMap.Drivetrain.VY_STAGE_kP, 0, 0);
    private static PIDController omegaStageController = new PIDController(RobotMap.Drivetrain.OMEGA_STAGE_kP, 0, 0);

    private final Timer timer = new Timer();

    public AlignToStage() {
        addRequirements(Drivetrain.getInstance());
        vxStageController.setTolerance(RobotMap.Drivetrain.MAX_ERROR_DEG_TX_STAGE);
        vyStageController.setTolerance(RobotMap.Drivetrain.MAX_ERROR_DEG_TY_STAGE);
        vxStageController.setSetpoint(0);
        vyStageController.setSetpoint(RobotMap.Drivetrain.VERTICAL_DEG_STAGE);
    }

    @Override
    public void initialize() {
        vxStageController.reset();
        vyStageController.reset();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (Limelight.atStage())
        {
            double vx = vxStageController.calculate(Limelight.getTx());
            double vy = -vyStageController.calculate(Limelight.getTy());
            // double vy = 0;

            Drivetrain.getInstance().setAngleAndDrive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0, Drivetrain.getInstance().getRotation()));
    
        }   
    }

    @Override
    public boolean isFinished() {
        return (vxStageController.atSetpoint() && vyStageController.atSetpoint()) ||
            timer.get() >= 1;
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }
}
