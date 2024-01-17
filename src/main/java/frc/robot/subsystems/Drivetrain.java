package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;

    private SwerveModule[] swerveModules;

    private Pigeon2 pigeon;
    private double prevHeading;

    private SwerveDriveKinematics kinematics; // converts chassis speeds (x, y, theta) to module states (speed, angle)

    public ProfiledPIDController thetaController = new ProfiledPIDController(RobotMap.Drivetrain.THETA_P, RobotMap.Drivetrain.THETA_I, RobotMap.Drivetrain.THETA_D, new Constraints(4, 3.5));

    // Estimates the robot's pose through encoder (state) and vision measurements;
    private SwerveDrivePoseEstimator poseEstimator;

    // Standard deviations of pose estimate (x, y, heading)
    private static Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.2); // increase to trust encoder (state) measurements less
    private static Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.05, 0.025, 0.05); // increase to trust vsion measurements less

    private Drivetrain() {
        // initialize swerve modules
        // SmartDashboard.putNumber("TranslationkP", RobotMap.SwerveModule.TRANSLATION_KP);
        // SmartDashboard.putNumber("TranslationkI", RobotMap.SwerveModule.TRANSLATION_KI);
        // SmartDashboard.putNumber("TranslationkD", RobotMap.SwerveModule.TRANSLATION_KD);
        // SmartDashboard.putNumber("TranslationkP", RobotMap.SwerveModule.ROTATION_KP);
        swerveModules =
            new SwerveModule[] {
                new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
            };

        // initialize pigeon
        pigeon = new Pigeon2(RobotMap.Drivetrain.PIGEON_ID);
        initPigeon();

        // initialize locations of swerve modules relative to robot (fl, fr, bl, br)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2));

        // sets how much error to allow on theta controller
        thetaController.setTolerance(RobotMap.Drivetrain.MAX_ERROR_YAW);
        // SmartDashboard.putData("Rotation PID", thetaController);
        // SmartDashboard.putNumber("kP", thetaController.getP());
        // SmartDashboard.putNumber("kI", thetaController.getI());
        // SmartDashboard.putNumber("kD", thetaController.getD());

        // initial pose (holds the x, y, heading)
        Pose2d initalPoseMeters = new Pose2d();

        // initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getRotation(),
            getModulePositions(),
            initalPoseMeters,
            stateStdDevs,
            visionStdDevs);
    }

    /*
     * Initialize pigeon values
     */
    private void initPigeon() {
        Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();
        pigeonConfigs.MountPose.MountPosePitch = 0;
        pigeonConfigs.MountPose.MountPoseRoll = 0;
        pigeonConfigs.MountPose.MountPoseYaw = 0;
        pigeonConfigs.Pigeon2Features.EnableCompass = false;
        pigeon.getConfigurator().apply(pigeonConfigs);
        pigeon.setYaw(0);
    }

    /**
     * Updates previous heading to the current heading or continues in the
     * same direction (omega becomes adjusted by the prevous heading)
     * @param omega rotational speed
     * @return      adjusted rotational speed
     */
    public double adjustPigeon(double omega) {
        if (Math.abs(omega) <= RobotMap.Drivetrain.MIN_OUTPUT) {
            omega = -RobotMap.Drivetrain.PIGEON_kP * (prevHeading - getHeading());
        }
        else {
            prevHeading = getHeading();
        }
    
        return omega;
      }

    /*
     * Returns yaw of pigeon in degrees (heading of robot)
     */
    public double getHeading() {
        double yaw = pigeon.getYaw().getValueAsDouble();
        //SmartDashboard.putNumber("pigeon heading", pigeon.getYaw());
        return yaw;
    }

    /**
     * @return pitch of pigeon in degrees
     */
    public double getPitch() {
        double pitch = pigeon.getYaw().getValueAsDouble();
        return pitch;
    }

    public void setTranslationkP(double newkP) {
        SmartDashboard.putNumber("newTranslationkP", newkP);
        for(int i = 0; i<4;i++) {
            swerveModules[i].setTranslationkP(newkP);
        }
    }

    public void setTranslationkI(double newkI) {
        SmartDashboard.putNumber("newTranslationkI", newkI);
        for(int i = 0; i<4;i++) {
            swerveModules[i].setTranslationkP(newkI);
        }
    }

    public void setTranslationkD(double newkD) {
        SmartDashboard.putNumber("newTranslationkD", newkD);
        for(int i = 0; i<4;i++) {
            swerveModules[i].setTranslationkP(newkD);
        }
    }

    public void setRotationkP(double newkP) {
        SmartDashboard.putNumber("newTranslationkP", newkP);
        for(int i = 0; i<4;i++) {
            swerveModules[i].setTranslationkP(newkP);
        }
    }

    /**
     * @return roll of pigeon in degrees
     */
    public double getRoll() {
        double roll = pigeon.getRoll().getValueAsDouble();
        return roll;
    }

    /**
     * @return heading of pigeon as a Rotation2d
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * @return the states of the swerve modules in an array
     */
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
          swerveModules[0].getSwerveModulePosition(),
          swerveModules[1].getSwerveModulePosition(),
          swerveModules[2].getSwerveModulePosition(),
          swerveModules[3].getSwerveModulePosition()
        };
    }

    /**
     * Sets the initial pose of the drivetrain
     * @param pose      intial Pose2d of drivetrain
     */
    public void setPose(Pose2d pose) {
        swerveModules[0].zeroTranslation();
        swerveModules[1].zeroTranslation();
        swerveModules[2].zeroTranslation();
        swerveModules[3].zeroTranslation();
        setYaw(pose.getRotation().getDegrees());
        poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }

    /**
     * Sets the yaw of the pigeon to the given angle
     * @param yaw   angle in degrees
     */
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
        setPreviousHeading(yaw);
    }

    /**
     * Updates previous heading
     * @param prev new heading
     */
    public void setPreviousHeading(double prev) {
        prevHeading = prev;
    }

    /**
     * Uses CameraPoseEstimation to align to the detected target and returns
     * the needed angle to adjust to the target
     * @param omega     rotational speed
     * @return          adjusted rotational speed
     */
    // public double alignToTarget(double omega) {
    //     var result = CameraPoseEstimation.getInstance().getCamera().getLatestResult();
    //     if (result.hasTargets()) {
    //     omega =
    //     -thetaController.calculate(result.getBestTarget().getYaw() - RobotMap.Drivetrain.OFFSET);
    // setPreviousHeading(getHeading());
    //     }
    //     return omega;
    //   }

    /**
     * @return kinematics of swerve drive
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Singleton code
     * @return instance of Drivetrain
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    /**
     * Converts chassis speeds to individual swerve module 
     * states and sets the angle and drive for them
     * @param chassis       chassis speeds to convert
     */
    public void setAngleAndDrive(ChassisSpeeds chassis) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassis);
        swerveModules[0].setAngleAndDrive(states[0]);
        swerveModules[1].setAngleAndDrive(states[1]);
        swerveModules[2].setAngleAndDrive(states[2]);
        swerveModules[3].setAngleAndDrive(states[3]);
    }

    /**
     * Called every loop, feeds newest encoder readings to estimator
     */
    public void updatePose() {
        poseEstimator.update(getRotation(), getModulePositions());
    }

    /**
     * @return the estimated pose aas a Pose2d
     */
    public Pose2d getPoseEstimatorPose2d() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        updatePose();
    }

//     @Override
//     public void initSendable(SendableBuilder builder) {
//         builder.setSmartDashboardType("Drivetrain");
//         builder.setActuator(true);
//         builder.setSafeState(() -> setAngleAndDrive(new ChassisSpeeds()));
//         builder.addDoubleProperty("Pitch Value", () -> getPitch(), null);
//         builder.addDoubleProperty("Roll Value", () -> getRoll(), null);

//         for (int i = 0; i < 4; i++) {
//         builder.addDoubleProperty(
//             SwerveModule.swerveIDToName(i) + " Translation Speed", swerveModules[i]::getSpeed, null);
//         builder.addDoubleProperty(
//             SwerveModule.swerveIDToName(i) + " Translation Position",
//             swerveModules[i]::getWheelPosition,
//             null);
//         builder.addDoubleProperty(
//             SwerveModule.swerveIDToName(i) + " Rotation Angle", swerveModules[i]::getAngle, null);
//         }
//     }
}
