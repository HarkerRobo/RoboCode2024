package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

import java.util.ArrayList;
import java.util.List;

public class Trajectories {
        /**
         * Three Note Path (Bottom)
         */
        public static Trajectory startToNote1_three = generateTrajectory(
                List.of(
                        new Pose2d(1.51, 1.36, Rotation2d.fromDegrees(180)),
                        new Pose2d(8.02, 0.64, Rotation2d.fromDegrees(180))),
            4.0,
            2.0,
            0.0,
            0.0,
            true);
        public static Trajectory note1ToShooter_three = generateTrajectory(
                List.of(new Pose2d(8.02, 0.64, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.52, 1.17, Rotation2d.fromDegrees(180))),
                2.0,
                1.0,
                0.0,
                0.0,
                false);
        public static Trajectory shooterToNote2_three = generateTrajectory(
                List.of(new Pose2d(5.52, 1.17, Rotation2d.fromDegrees(180)),
                        new Pose2d(8.02, 2.48, Rotation2d.fromDegrees(180))),
                2.0,
                1.0,
                0.0,
                0.0,
                true);
        public static Trajectory note2ToShooter_three = generateTrajectory(
                List.of(new Pose2d(8.02, 2.48, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.52, 1.17, Rotation2d.fromDegrees(180))),
                2.0,
                2.0,
                0.0,
                0.0,
                false);

        /**
         * Four Note Path (Top)
         */
        public static Trajectory startToNote1_four = generateTrajectory(
                List.of(new Pose2d(1.28, 5.41, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.58, 4.03, Rotation2d.fromDegrees(156.57))),
                2.0, 
                1.0, 
                0.0, 
                0.0,
                true);
        public static Trajectory note1Tonote2_four = generateTrajectory(
                List.of(new Pose2d(2.58, 4.03, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.73, 5.54, Rotation2d.fromDegrees(180))),
                1.0, 
                0.5, 
                0, 
                0, 
                false);
        public static Trajectory note2ToNote3_four = generateTrajectory(
                List.of(new Pose2d(2.73, 5.54, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.73, 7.06, Rotation2d.fromDegrees(-150))),
                1.0,
                0.5,
                0.0,
                0.0,
                false);

        /**
         * Six Note Path (Top)
         */
        public static Trajectory note3ToNote4_six = generateTrajectory(
                List.of(new Pose2d(2.73, 7.06, Rotation2d.fromDegrees(-150)),
                        new Pose2d(7.97, 5.79, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);
        public static Trajectory note4ToShooter_six = generateTrajectory(
                List.of(new Pose2d(7.97, 5.79, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.82, 6.42, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);
        public static Trajectory shooterToNote5_six = generateTrajectory(
                List.of(new Pose2d(5.82, 6.42, Rotation2d.fromDegrees(180)),
                        new Pose2d(7.97, 7.47, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);
        public static Trajectory note5ToShooter_six = generateTrajectory(
                List.of(new Pose2d(7.97, 7.47, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.37, 6.58, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);

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
}