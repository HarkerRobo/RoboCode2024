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
                        new Pose2d(1.51, 1.36, Rotation2d.fromDegrees(-150)),
                        new Pose2d(7.75, 0.78, Rotation2d.fromDegrees(180))),
            4.0,
            2.0,
            0.0,
            0.0,
            true);
        public static Trajectory note1ToMiddle_three = generateTrajectory(
                List.of(new Pose2d(7.75, 0.78, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.48, 1.95, Rotation2d.fromDegrees(180))),
                2.0,
                1.0,
                0.0,
                0.0,
                false);
        public static Trajectory middleToShooter_three = generateTrajectory(
                List.of(new Pose2d(5.48, 1.95, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.88, 3.42, Rotation2d.fromDegrees(-38.68))),
                2.0,
                1.0,
                0.0,
                0.0,
                true);
        public static Trajectory shooterToMiddle2_three = generateTrajectory(
                List.of(new Pose2d(2.88, 3.42, Rotation2d.fromDegrees(-38.68)),
                        new Pose2d(5.48, 1.95, Rotation2d.fromDegrees(180))),
                2.0,
                2.0,
                0.0,
                0.0,
                false);
        public static Trajectory middleToNote2_three = generateTrajectory(
                List.of(new Pose2d(5.48, 1.95, Rotation2d.fromDegrees(180)),
                        new Pose2d(8.02, 2.48, Rotation2d.fromDegrees(180))),
                2.0,
                2.0,
                0.0,
                0.0,
                false);
        public static Trajectory note2ToMiddle_three = generateTrajectory(
                List.of(new Pose2d(8.02, 2.48, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.48, 1.95, Rotation2d.fromDegrees(180))),
                2.0,
                2.0,
                0.0,
                0.0,
                false);
        public static Trajectory middleToShooter2_three = generateTrajectory(
                List.of(new Pose2d(5.48, 1.95, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.88, 3.42, Rotation2d.fromDegrees(-38.70))),
                2.0,
                1.0,
                0.0,
                0.0,
                false);

        /**
         * Four Note Path (Top)
         */
        public static Trajectory startToNote1_four = generateTrajectory(
                List.of(new Pose2d(1.28, 5.41, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.57, 4.15, Rotation2d.fromDegrees(180))),
                2.0, 
                1.0, 
                0.0, 
                0.0,
                true);
        public static Trajectory note1ToShoot1_four = generateTrajectory(
                List.of(new Pose2d(2.57, 4.15, Rotation2d.fromDegrees(180)),
                        new Pose2d(1.72, 5.41, Rotation2d.fromDegrees(180))),
                1.0, 
                0.5, 
                0, 
                0, 
                false);
        public static Trajectory shoot1ToNote2_four = generateTrajectory(
                List.of(new Pose2d(1.72, 5.41, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.73, 5.54, Rotation2d.fromDegrees(180))),
                1.0, 
                0.5, 
                0, 
                0, 
                false);
        public static Trajectory note2ToShoot2_four = generateTrajectory(
                List.of(new Pose2d(2.73, 5.54, Rotation2d.fromDegrees(180)),
                        new Pose2d(1.84, 5.96, Rotation2d.fromDegrees(15.97))),
                1.0,
                0.5,
                0.0,
                0.0,
                false);
        public static Trajectory shoot2ToNote3_four = generateTrajectory(
                List.of(new Pose2d(1.84, 5.96, Rotation2d.fromDegrees(15.97)),
                        new Pose2d(2.73, 7.06, Rotation2d.fromDegrees(180))),
                1.0,
                0.5,
                0.0,
                0.0,
                false);
        public static Trajectory note3ToShoot3_four = generateTrajectory(
                List.of(new Pose2d(2.73, 7.06, Rotation2d.fromDegrees(180)),
                        new Pose2d(1.84, 6.43, Rotation2d.fromDegrees(35.80))),
                1.0,
                0.5,
                0.0,
                0.0,
                false);

        /**
         * Six Note Path (Top)
         */
        public static Trajectory startToNote1_six = generateTrajectory(
                List.of(new Pose2d(1.72, 5.56, Rotation2d.fromDegrees(-150)),
                        new Pose2d(2.56, 4.16, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);
        public static Trajectory note1ToShoot1_six = generateTrajectory(
                List.of(new Pose2d(2.56, 4.16, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.28, 4.89, Rotation2d.fromDegrees(-17.9))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);
        public static Trajectory shoot1ToNote2_six = generateTrajectory(
                List.of(new Pose2d(2.28, 4.89, Rotation2d.fromDegrees(-17.9)),
                        new Pose2d(2.41, 5.56, Rotation2d.fromDegrees(-2.47))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);
        public static Trajectory note2ToShoot2_six = generateTrajectory(
                List.of(new Pose2d(2.41, 5.56, Rotation2d.fromDegrees(-2.47)),
                        new Pose2d(2.10, 6.15, Rotation2d.fromDegrees(23.3))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);
        public static Trajectory note5ToShooter_six = generateTrajectory(
                List.of(new Pose2d(2.10, 6.15, Rotation2d.fromDegrees(23.3)),
                        new Pose2d(2.56, 7.01, Rotation2d.fromDegrees(0))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);
        public static Trajectory shoot2ToNote3_six = generateTrajectory(
                List.of(new Pose2d(2.56, 7.01, Rotation2d.fromDegrees(0)),
                        new Pose2d(5.37, 6.58, Rotation2d.fromDegrees(23.3))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);
        public static Trajectory note3ToShoot3_six = generateTrajectory(
                List.of(new Pose2d(5.37, 6.58, Rotation2d.fromDegrees(23.3)),
                        new Pose2d(5.79, 6.34, Rotation2d.fromDegrees(0))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);
        public static Trajectory shoot3ToNote4_six = generateTrajectory(
                List.of(new Pose2d(5.79, 6.34, Rotation2d.fromDegrees(0)),
                        new Pose2d(7.97,5.79, Rotation2d.fromDegrees(0))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);
        public static Trajectory note4ToShoot4_six = generateTrajectory(
                List.of(new Pose2d(7.97, 5.79, Rotation2d.fromDegrees(0)),
                        new Pose2d(5.82,6.41, Rotation2d.fromDegrees(15.8))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);
        public static Trajectory shoot4ToNote5_six = generateTrajectory(
                List.of(new Pose2d(5.82, 6.41, Rotation2d.fromDegrees(15.8)),
                        new Pose2d(7.97,7.47, Rotation2d.fromDegrees(0))),
            2.0,
            1.0,
            0.0,
            0.0,
            false);
        public static Trajectory note5ToShoot5_six = generateTrajectory(
                List.of(new Pose2d(7.97, 7.47, Rotation2d.fromDegrees(0)),
                        new Pose2d(5.37,6.58, Rotation2d.fromDegrees(0))),
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