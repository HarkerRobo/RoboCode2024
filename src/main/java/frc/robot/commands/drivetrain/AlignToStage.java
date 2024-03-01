package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.Flip;
import frc.robot.util.Limelight;

public class AlignToStage extends Command {

    private static PIDController vxStageController = new PIDController(RobotMap.Drivetrain.VX_STAGE_kP, 0, 0);
    private static PIDController vyStageController = new PIDController(RobotMap.Drivetrain.VY_STAGE_kP, 0, 0);
    private static PIDController omegaStageController = new PIDController(RobotMap.Drivetrain.OMEGA_STAGE_kP, 0, 0);

    private final Timer timer = new Timer();

    private String name;

    public AlignToStage(String side) {
        this.name = side;
        addRequirements(Drivetrain.getInstance());
        vxStageController.setTolerance(RobotMap.Drivetrain.MAX_ERROR_DEG_TX_STAGE);
        vyStageController.setTolerance(RobotMap.Drivetrain.MAX_ERROR_DEG_TY_STAGE);
        vxStageController.setSetpoint(0);
        vyStageController.setSetpoint(RobotMap.Drivetrain.VERTICAL_DEG_STAGE);
    }

    public double[] alignToStage(String side) {

        double refXFieldRel = 0;
        double refYFieldRel = 0;

        switch (side) {
            case "center":
                refXFieldRel = Flip.apply(RobotMap.Field.STAGE_CENTER)
                    .minus(Drivetrain.getInstance().getPoseEstimatorPose2d().getTranslation()).getX();
                refYFieldRel = Flip.apply(RobotMap.Field.STAGE_CENTER)
                    .minus(Drivetrain.getInstance().getPoseEstimatorPose2d().getTranslation()).getY();
                break;

            case "left":
                refXFieldRel = Flip.apply(RobotMap.Field.STAGE_LEFT)
                    .minus(Drivetrain.getInstance().getPoseEstimatorPose2d().getTranslation()).getX();
                refYFieldRel = Flip.apply(RobotMap.Field.STAGE_LEFT)
                    .minus(Drivetrain.getInstance().getPoseEstimatorPose2d().getTranslation()).getY();
                break;

            case "right":
                refXFieldRel = Flip.apply(RobotMap.Field.STAGE_RIGHT)
                    .minus(Drivetrain.getInstance().getPoseEstimatorPose2d().getTranslation()).getX();
                refYFieldRel = Flip.apply(RobotMap.Field.STAGE_RIGHT)
                    .minus(Drivetrain.getInstance().getPoseEstimatorPose2d().getTranslation()).getY();
                break;
        
            default:
                break;
        }

        double vx = MathUtil.clamp(vxStageController.calculate(refXFieldRel, 0), -1, 1);
        double vy = MathUtil.clamp(vyStageController.calculate(refYFieldRel, 0), -1, 1);
        double omega = MathUtil.clamp(omegaStageController.calculate(Drivetrain.getInstance().getPoseEstimatorPose2d().getRotation().getRadians()), -1, 1);

        return new double[] {vx, vy, omega};
    }

    @Override
    public void initialize() {
        // vxStageController.reset();j
        // vyStageController.reset();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Drivetrain.getInstance().setAngleAndDrive(ChassisSpeeds.fromFieldRelativeSpeeds(alignToStage(name)[0], alignToStage(name)[1], -alignToStage(name)[2], Drivetrain.getInstance().getRotation()));
    }

    @Override
    public boolean isFinished() {
        return (vxStageController.atSetpoint() && vyStageController.atSetpoint() && omegaStageController.atSetpoint()) ||
            timer.get() >= 1;
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }
}
