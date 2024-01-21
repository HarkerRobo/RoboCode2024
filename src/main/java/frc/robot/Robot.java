// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.Trajectories;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;

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

  @Override
  public void robotInit() {
    LiveWindow.setEnabled(true);
    LiveWindow.enableAllTelemetry();
    SmartDashboard.putData(RobotMap.Field.FIELD);
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManual());
    autonChooser = new SendableChooser<String>();
    autonChooser.setDefaultOption("Three Note Path Top", "Three Note Path Top");
    autonChooser.addOption("Six Note Path Top", "Six Note Path Top");
    autonChooser.addOption("Two Note Path Bottom", "Two Note Path Bottom");
    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //RobotMap.Field.FIELD.setRobotPose(Drivetrain.getInstance().getPoseEstimatorPose2d());

    SmartDashboard.putString("Current Auton:", autonChooser.getSelected());
    SmartDashboard.putBoolean("isTargetVisible", Limelight.isTargetVisible());


    SmartDashboard.putNumber("tx", Limelight.getTx());
    SmartDashboard.putNumber("t y", Limelight.getTy());

    SmartDashboard.putNumber("target dist",Limelight.getTargetDistance());
    SmartDashboard.putNumberArray("pose", new Double[] {Limelight.getTargetDistance(), Limelight.getTargetAngle()});

    NetworkTableInstance.getDefault().flushLocal();
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void autonomousInit() {
    // switch (autonChooser.getSelected()) {
    //   case "Three Note Path Top":
    //     Drivetrain.getInstance().setPose(Trajectories.apply(new Pose2d(1.64, 5.54, Rotation2d.fromDegrees(180))));
    //   case "Six Note Path Top":
    //     Drivetrain.getInstance().setPose(Trajectories.apply(new Pose2d(1.72, 5.56, Rotation2d.fromDegrees(180))));
    //   case "Two Note Path Bottom":
    //     Drivetrain.getInstance().setPose(Trajectories.apply(new Pose2d(1.19, 1.84, Rotation2d.fromDegrees(180))));
    // }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //Drivetrain.getInstance().setYaw(180);
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}