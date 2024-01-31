// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.commands.CommandGroups;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.Flip;
import frc.robot.util.Limelight;
import frc.robot.util.Telemetry;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private SendableChooser<String> autonChooser;
  private Telemetry telemetry;

  @Override
  public void robotInit() {
    LiveWindow.setEnabled(true);
    LiveWindow.enableAllTelemetry();
    SmartDashboard.putData(RobotMap.Field.FIELD);
    Limelight.setCameraPose(RobotMap.Camera.FORWARD, RobotMap.Camera.UP, RobotMap.Camera.PITCH);
    // CommandScheduler.getInstance().schedule(CommandGroups.FULL_ZERO);

    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManual());

    telemetry = new Telemetry();
    telemetry.swerve();

    autonChooser = new SendableChooser<String>();
    autonChooser.setDefaultOption("Three Note Path Top", "Three Note Path Top");
    autonChooser.addOption("Six Note Path Top", "Six Note Path Top");
    autonChooser.addOption("Two Note Path Bottom", "Two Note Path Bottom");
    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotMap.Field.FIELD.setRobotPose(Drivetrain.getInstance().getPoseEstimatorPose2d());

    SmartDashboard.putString("Current Auton:", autonChooser.getSelected());

    telemetry.publish();

    NetworkTableInstance.getDefault().flushLocal();
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void autonomousInit() {
    switch (autonChooser.getSelected()) {
      case "Four Note Path Top":
        Drivetrain.getInstance().setPose(Flip.apply(new Pose2d(1.28, 5.41, Rotation2d.fromDegrees(180))));
        break;
      case "Three Note Path Bottom":
        Drivetrain.getInstance().setPose(Flip.apply(new Pose2d(1.51, 1.36, Rotation2d.fromDegrees(180))));
        break;
      default:
        Drivetrain.getInstance().setPose(Flip.apply(new Pose2d(1.28, 5.41, Rotation2d.fromDegrees(180))));
        break;
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Drivetrain.getInstance().setPose(Flip.apply(new Pose2d())); // only for tuning
  }

  @Override
  public void teleopPeriodic() {
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds(0, 0,0));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}