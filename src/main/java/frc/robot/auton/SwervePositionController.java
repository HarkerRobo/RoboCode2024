package frc.robot.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.Flip;

import java.util.function.Supplier;

public class SwervePositionController extends Command {

    /**
     * PID Controllers for X, Y, and Rotation (THETA)
     */

    public static PIDController xController = new PIDController(RobotMap.Drivetrain.X_kP, RobotMap.Drivetrain.X_kI, RobotMap.Drivetrain.X_kD);
    public static PIDController yController = new PIDController(RobotMap.Drivetrain.Y_kP, RobotMap.Drivetrain.Y_kI, RobotMap.Drivetrain.Y_kD);
    public static PIDController thetaController = new PIDController(RobotMap.Drivetrain.THETA_kP, RobotMap.Drivetrain.THETA_kI, RobotMap.Drivetrain.THETA_kD);

    private Trajectory trajectory;
    private Supplier<Rotation2d> referenceHeading;
    private boolean alignToSpeaker;

    private final Timer timer = new Timer();
    
    public SwervePositionController(
            Trajectory trajectory, Supplier<Rotation2d> referenceHeading, boolean alignToSpeaker) {
        this.trajectory = trajectory;
        this.referenceHeading = referenceHeading;
        this.alignToSpeaker = alignToSpeaker;
        thetaController.enableContinuousInput(-0.5, 0.5);
        addRequirements(Drivetrain.getInstance());
    }

    public SwervePositionController(
            Trajectory trajectory, Supplier<Rotation2d> referenceHeading) {
        this(trajectory, referenceHeading, false);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        // xController.setP(RobotMap.Drivetrain.X_kP);
        // yController.setP(RobotMap.Drivetrain.Y_kP);
        // thetaController.setP(RobotMap.Drivetrain.THETA_kP);

        Trajectory.State desiredState = Flip.apply(trajectory.sample(timer.get()));
        Rotation2d referenceAngle = Flip.apply(referenceHeading.get());

        Pose2d currentPose = Drivetrain.getInstance().getPoseEstimatorPose2d();
        Rotation2d currentRotation = currentPose.getRotation();

        double clampAdd = 1 + Math.abs(referenceAngle.getRotations() - currentRotation.getRotations());

        /**
         * Feedforward values for X, Y, and Rotation (THETA)
         */

        double xFF = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getCos(); // meters per second
        double yFF = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getSin(); // meters per second

        double thetaFF = MathUtil.clamp((alignToSpeaker) ? -Drivetrain.getInstance().alignToSpeaker()
                                            : -thetaController.calculate(currentRotation.getRotations(), referenceAngle.getRotations()),
                                            -clampAdd, clampAdd); // radians per second

        /**
         * Feedback values for X, Y
         */
        double xFeedback = xController.calculate(currentPose.getX(), desiredState.poseMeters.getX());
        double yFeedback = yController.calculate(currentPose.getY(), desiredState.poseMeters.getY());

        /**
         * Send values to Drivetrain
         */
        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, thetaFF,
                Drivetrain.getInstance().getPoseEstimatorPose2d().getRotation());

        Drivetrain.getInstance().setAngleAndDrive(adjustedSpeeds);
    }

    /**
     * Check if the robot has reached the end of the trajectory
     */

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }

    /**
     * @param interrupted
     * @return set drivetrain to 0
     */

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }
}