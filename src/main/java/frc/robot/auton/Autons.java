package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups;
// import frc.robot.commands.CommandGroups;
// import frc.robot.commands.indexer.IndexToShooter;
// import frc.robot.commands.shooter.MoveNoteToShooter;
// import frc.robot.commands.shooter.ShooterManual;
import edu.wpi.first.math.geometry.Rotation2d;


public class Autons
{
    public static final SequentialCommandGroup fourNotePath = new SequentialCommandGroup(
        // CommandGroups.getFullZeroCommand(),
        new SwervePositionController(Trajectories.startToNote1_four, () -> Rotation2d.fromDegrees(180), false),//.alongWith(CommandGroups.getFullShootSpeaker()),
        // CommandGroups.getFullIntakeCommand(),
        new SwervePositionController(Trajectories.note1ToShoot1_four, () -> Rotation2d.fromDegrees(180), false),//.alongWith(CommandGroups.getFullShootSpeaker()),
        new SwervePositionController(Trajectories.shoot1ToNote2_four, () -> Rotation2d.fromDegrees(180), false),
        // CommandGroups.getFullIntakeCommand(),
        new SwervePositionController(Trajectories.note2ToShoot2_four, () -> Rotation2d.fromDegrees(180), false),//.alongWith(CommandGroups.getFullShootSpeaker()),
        new SwervePositionController(Trajectories.shoot2ToNote3_four, () -> Rotation2d.fromDegrees(-157.83), false),
        // CommandGroups.getFullIntakeCommand(),
        new SwervePositionController(Trajectories.note3ToShoot3_four, () -> Rotation2d.fromDegrees(-136.16), false)//.alongWith(CommandGroups.getFullShootSpeaker())
    );

    public static final SequentialCommandGroup threeNotePath = new SequentialCommandGroup(
        
        // new SwervePositionController(Trajectories.startToShoot1_three, () -> Rotation2d.fromDegrees(180), true).alongWith(CommandGroups.FULL_SHOOT_SPEAKER),
        // CommandGroups.FULL_INTAKE,
        // new SwervePositionController(Trajectories.shoot1ToMiddle1_three, () -> Rotation2d.fromDegrees(140.1), false),
        // new SwervePositionController(Trajectories.middleToNote1_three, () -> Rotation2d.fromDegrees(180), false),
        // CommandGroups.FULL_INTAKE,
        // new SwervePositionController(Trajectories.noteToMiddle2_three, () -> Rotation2d.fromDegrees(180), false),
        // new SwervePositionController(Trajectories.middle2ToShoot2_three, () -> Rotation2d.fromDegrees(180), true).alongWith(CommandGroups.FULL_SHOOT_SPEAKER),
        // new SwervePositionController(Trajectories.shoot2ToNote2_three, () -> Rotation2d.fromDegrees(138.59), false),
        // CommandGroups.FULL_INTAKE,
        // new SwervePositionController(Trajectories.note2ToShoot3_three, () -> Rotation2d.fromDegrees(180), true).alongWith(CommandGroups.FULL_SHOOT_SPEAKER)

    );
    public static final SequentialCommandGroup sixNotePath = new SequentialCommandGroup
    (
        // new SwervePositionController(Trajectories.startToNote1_six, () -> Rotation2d.fromDegrees(180), false),
        // new SwervePositionController(Trajectories.note1ToShoot1_six, () -> Rotation2d.fromDegrees(180), true),
        // new SwervePositionController(Trajectories.shoot1ToNote2_six, () -> Rotation2d.fromDegrees(158.62), false),
        // new SwervePositionController(Trajectories.note2ToShoot2_six, () -> Rotation2d.fromDegrees(178.45), true),
        // new SwervePositionController(Trajectories.note5ToShooter_six, () -> Rotation2d.fromDegrees(-160.76), true),
        // new SwervePositionController(Trajectories.shoot2ToNote3_six, () -> Rotation2d.fromDegrees(180), false),
        // new SwervePositionController(Trajectories.note3ToShoot3_six, () -> Rotation2d.fromDegrees(-156.72), true),
        // new SwervePositionController(Trajectories.shoot3ToNote4_six, () -> Rotation2d.fromDegrees(180), false),
        // new SwervePositionController(Trajectories.note4ToShoot4_six, () -> Rotation2d.fromDegrees(-164.18), true),
        // new SwervePositionController(Trajectories.shoot4ToNote5_six, () -> Rotation2d.fromDegrees(180), false),
        // new SwervePositionController(Trajectories.note5ToShoot5_six, () -> Rotation2d.fromDegrees(-162.5), true)
    );
}