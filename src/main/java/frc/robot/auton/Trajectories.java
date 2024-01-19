package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap;

import java.util.ArrayList;
import java.util.List;

public class Trajectories {
    /**
     * Two Note Path (Bottom)
     */

    public static Trajectory twoNoteBottom = generateTrajectory(
            List.of(
                    new Pose2d(1.19, 1.84, Rotation2d.fromDegrees(180)),
                    new Pose2d(8.02, 0.64, Rotation2d.fromDegrees(180)),
                    new Pose2d(5.52, 1.17, Rotation2d.fromDegrees(180)),
                    new Pose2d(8.02, 2.48, Rotation2d.fromDegrees(180)),
                    new Pose2d(5.52, 1.17, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);

    /**
     * Three Note Path (Top)
     */

    public static Trajectory threeNoteTop = generateTrajectory(
            List.of(
                    new Pose2d(1.64, 5.54, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.58, 4.03, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.73, 5.54, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.73, 6.83, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.73, 7.06, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);

    /**
     * Six Note Path (Top)
     */

    public static Trajectory sixNoteTop = generateTrajectory(
            List.of(
                    new Pose2d(1.72, 5.56, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.56, 4.16, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.56, 5.56, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.56, 7.01, Rotation2d.fromDegrees(180)),
                    new Pose2d(7.97, 5.79, Rotation2d.fromDegrees(180)),
                    new Pose2d(5.82, 6.42, Rotation2d.fromDegrees(180)),
                    new Pose2d(7.97, 7.47, Rotation2d.fromDegrees(180)),
                    new Pose2d(5.60, 6.42, Rotation2d.fromDegrees(180)),
                    new Pose2d(7.97, 4.16, Rotation2d.fromDegrees(180)),
                    new Pose2d(5.37, 6.58, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);


    /**
     * generates a Trajectory given a list of Pose2d points, max velocity, max
     * acceleration, start velocity, and end velocity, and if flipped due to
     * alliance
     */

    public static Trajectory generateTrajectory(
            List<Pose2d> waypoints,
            double maxVelocity,
            double maxAcceleration,
            double startVelocity,
            double endVelocity,
            boolean reversed,
            TrajectoryConstraint... constraints) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);

        for (TrajectoryConstraint constraint : constraints) {
            config.addConstraint(constraint);
        }

        config.setStartVelocity(startVelocity);
        config.setEndVelocity(endVelocity);
        config.setReversed(reversed);

        // interiorPoints - points between the two end points along auton path
        List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
        for (int i = 1; i < waypoints.size() - 1; i++) {
            interiorPoints.add(waypoints.get(i).getTranslation());
        }

        return TrajectoryGenerator.generateTrajectory(
                waypoints.get(0), interiorPoints, waypoints.get(waypoints.size() - 1), config);
    }

    /**
     * Flips the Translation2d based on current alliance
     */

    public static Translation2d apply(Translation2d translation2d) {
        return (isFlipped())
                ? new Translation2d(RobotMap.Field.fieldLength - translation2d.getX(),
                        translation2d.getY())
                : translation2d;
    }

    /**
     * Flips the Rotation2d based on current alliance
     */

    public static Rotation2d apply(Rotation2d rotation2d) {
        return (isFlipped()) ? new Rotation2d(-rotation2d.getCos(), rotation2d.getSin()) : rotation2d;
    }

    /**
     * Flips the Pose2d based on current alliance
     */

    public static Pose2d apply(Pose2d pose2d) {
        return (isFlipped())
                ? new Pose2d(RobotMap.Field.fieldLength - pose2d.getX(), pose2d.getY(),
                        new Rotation2d(-pose2d.getRotation().getCos(), pose2d.getRotation().getSin()))
                : pose2d;
    }

    /**
     * Flips the Trajectory.State based on current alliance
     */

    public static Trajectory.State apply(Trajectory.State state) {
        return (isFlipped()) ? new Trajectory.State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                new Pose2d(
                        RobotMap.Field.fieldLength - state.poseMeters.getX(),
                        state.poseMeters.getY(),
                        new Rotation2d(
                                -state.poseMeters.getRotation().getCos(),
                                state.poseMeters.getRotation().getSin())),
                -state.curvatureRadPerMeter) : state;
    }

    /**
     * if Alliance is red, flip the trajectory
     * field flipped along y-axis
     */
    public static boolean isFlipped() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }
}