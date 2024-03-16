package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public class Trajectories {
        /**
         * Three Note Path (Bottom)
         */
        public static Trajectory startToShoot1_three = generateTrajectory(
                List.of(
                        new Pose2d(1.51, 1.36 + Units.inchesToMeters(3), Rotation2d.fromDegrees(180)),
                        new Pose2d(1.98, 3.48, Rotation2d.fromDegrees(140.1))),
            5.0,
            2.5,
            0.0,
            0.0,
            true);
        public static Trajectory shoot1ToMiddle1_three = generateTrajectory(
                List.of(new Pose2d(1.98, 3.48, Rotation2d.fromDegrees(140.1)),
                        new Pose2d(5.18, 1.87, Rotation2d.fromDegrees(180))),
                5.0,
                2.5,
                0.0,
                3.0,
                false);
        public static Trajectory middleToNote1_three = generateTrajectory(
                List.of(new Pose2d(5.18, 1.87, Rotation2d.fromDegrees(180)),
                        new Pose2d(7.82,2.35 , Rotation2d.fromDegrees(180))),
                5.0,
                2.5,
                3.0,
                0.0,
                true);
        public static Trajectory noteToMiddle2_three = generateTrajectory(
                List.of(new Pose2d(7.82, 2.35, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.18, 1.87, Rotation2d.fromDegrees(180))),
                5.0,
                2.5,
                0.0,
                3.0,
                false);
        public static Trajectory middle2ToShoot2_three = generateTrajectory(
                List.of(new Pose2d(5.18, 1.87, Rotation2d.fromDegrees(180)),
                        new Pose2d(1.98, 3.48, Rotation2d.fromDegrees(140.1))),
                5.0,
                2.5,
                3.0,
                0.0,
                false);
        public static Trajectory shoot2ToNote2_three= generateTrajectory(
                List.of(new Pose2d(1.98, 3.48, Rotation2d.fromDegrees(140.1)),
                        new Pose2d(7.69, 0.74, Rotation2d.fromDegrees(180))),
                5.0,
                2.5,
                0.0,
                0.0,
                false);
        public static Trajectory note2ToShoot3_three = generateTrajectory(
                List.of(new Pose2d(7.69, 0.74, Rotation2d.fromDegrees(180)),
                        new Pose2d(1.98, 3.48, Rotation2d.fromDegrees(140.0))),
                5.0,
                2.5,
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
                2.0, 
                1.0, 
                0, 
                0, 
                false);
        public static Trajectory shoot1ToNote2_four = generateTrajectory(
                List.of(new Pose2d(1.72, 5.41, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.73, 5.54, Rotation2d.fromDegrees(180))),
                2.0, 
                1.0, 
                0, 
                0, 
                false);
        public static Trajectory note2ToShoot2_four = generateTrajectory(
                List.of(new Pose2d(2.73, 5.54, Rotation2d.fromDegrees(180)),
                        new Pose2d(1.84, 5.96, Rotation2d.fromDegrees(-157.83))),
                2.0,
                1.0,
                0.0,
                0.0,
                false);
        public static Trajectory shoot2ToNote3_four = generateTrajectory(
                List.of(new Pose2d(1.84, 5.96, Rotation2d.fromDegrees(-157.83)),
                        new Pose2d(2.73, 7.06, Rotation2d.fromDegrees(-136.16))),
                2.0,
                1.0,
                0.0,
                0.0,
                false);
        public static Trajectory note3ToShoot3_four = generateTrajectory(
                List.of(new Pose2d(2.73, 7.06, Rotation2d.fromDegrees(-136.16)),
                        new Pose2d(1.84, 6.43, Rotation2d.fromDegrees(-143.16))),
                2.0,
                1.0,
                0.0,
                0.0,
                false);

        public static Trajectory shoot3ToEnd_four = generateTrajectory(
                List.of(new Pose2d(1.84, 6.43, Rotation2d.fromDegrees(-143.16)),
                        new Pose2d(1.79, 5.43, Rotation2d.fromDegrees(180))),
                2.0,
                1.0,
                0.0,
                0.0,
                false);

        /**
         * Six Note Path (Top)
         */
        public static Trajectory startToNote1_six = generateTrajectory(
                List.of(new Pose2d(1.72, 5.56, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.56, 4.16, Rotation2d.fromDegrees(180))),
            5.0,
            2.5,
            0.0,
            0.0,
            true);
        public static Trajectory note1ToShoot1_six = generateTrajectory(
                List.of(new Pose2d(2.56, 4.16, Rotation2d.fromDegrees(180)),
                        new Pose2d(2.28, 4.89, Rotation2d.fromDegrees(158.62))),
            5.0,
            2.5,
            0.0,
            3.0,
            true);
        public static Trajectory shoot1ToNote2_six = generateTrajectory(
                List.of(new Pose2d(2.28, 4.89, Rotation2d.fromDegrees(158.62)),
                        new Pose2d(2.41, 5.56, Rotation2d.fromDegrees(178.45))),
            5.0,
            2.5,
            3.0,
            0.0,
            false);
        public static Trajectory note2ToShoot2_six = generateTrajectory(
                List.of(new Pose2d(2.41, 5.56, Rotation2d.fromDegrees(178.45)),
                        new Pose2d(2.10, 6.15, Rotation2d.fromDegrees(-160.76))),
            5.0,
            2.5,
            0.0,
            2.0,
            true);
        public static Trajectory note5ToShooter_six = generateTrajectory(
                List.of(new Pose2d(2.10, 6.15, Rotation2d.fromDegrees(-160.76)),
                        new Pose2d(2.56, 7.01, Rotation2d.fromDegrees(180))),
            5.0,
            2.5,
            2.0,
            2.0,
            false);
        public static Trajectory shoot2ToNote3_six = generateTrajectory(
                List.of(new Pose2d(2.56, 7.01, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.37, 6.58, Rotation2d.fromDegrees(-156.72))),
            5.0,
            2.5,
            2.0,
            0.0,
            false);
        public static Trajectory note3ToShoot3_six = generateTrajectory(
                List.of(new Pose2d(5.37, 6.58, Rotation2d.fromDegrees(-156.72)),
                        new Pose2d(5.79, 6.34, Rotation2d.fromDegrees(180))),
            5.0,
            2.5,
            0.0,
            3.0,
            false);
        public static Trajectory shoot3ToNote4_six = generateTrajectory(
                List.of(new Pose2d(5.79, 6.34, Rotation2d.fromDegrees(180)),
                        new Pose2d(7.97,5.79, Rotation2d.fromDegrees(-164.18))),
            5.0,
            2.5,
            3.0,
            0.0,
            false);
        public static Trajectory note4ToShoot4_six = generateTrajectory(
                List.of(new Pose2d(7.97, 5.79, Rotation2d.fromDegrees(-164.18)),
                        new Pose2d(5.82,6.41, Rotation2d.fromDegrees(180))),
            5.0,
            2.5,
            0.0,
            3.0,
            false);
        public static Trajectory shoot4ToNote5_six = generateTrajectory(
                List.of(new Pose2d(5.82, 6.41, Rotation2d.fromDegrees(180)),
                        new Pose2d(7.97,7.47, Rotation2d.fromDegrees(-162.5))),
            5.0,
            2.5,
            3.0,
            0.0,
            false);
        public static Trajectory note5ToShoot5_six = generateTrajectory(
                List.of(new Pose2d(7.97,7.47, Rotation2d.fromDegrees(-162.5)),
                        new Pose2d(5.37, 6.58, Rotation2d.fromDegrees(180))),
            5.0,
            2.5,
            0.0,
            0.0,
            false);

        public static Trajectory note1_one = generateTrajectory(
                List.of(new Pose2d(0.90, 6.45, Rotation2d.fromDegrees(-119.49)),
                        new Pose2d(4.31, 7.36, Rotation2d.fromDegrees(180))),
                2.0, 1.0, 0.0, 0.0, true);
        

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