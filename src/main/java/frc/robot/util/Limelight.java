package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap;

public final class Limelight {
    public static final String LIMELIGHT_TABLE_KEY = "limelight";
    public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY);

    public static Pose2d getBotPose2d() {
        return toPose2D(getBotPoseVal());
    }

    public static boolean isPoseValid(Pose2d botPose, Pose2d visionBot) {
        return visionBot.getTranslation().getDistance(botPose.getTranslation()) < RobotMap.Camera.MAX_ERROR_VISION_POSE;
    }

    public static double getTimestamp() {
        return Timer.getFPGATimestamp() - getBotPoseVal()[6] / 1000.0;
    }

    public static boolean hasTargets() {
        return MathUtil.compareDouble(TABLE.getEntry("tv").getDouble(0.0), 1.0);
    }

    public static boolean atAmp() {
        if (hasTargets()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_AMP_RED);
            } else {
                return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_AMP_BLUE);
            }
        }
        return false;
    }

    public static boolean atSpeaker() {
        if (hasTargets()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_SPEAKER_RED[0]) || MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_SPEAKER_RED[1]);
            } else {
                return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_SPEAKER_BLUE[0]) || MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_SPEAKER_BLUE[1]);
            }
        }
        return false;
    }

    public static boolean atStage() {
        if (hasTargets()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_RED[0]) || MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_RED[1]) || 
                MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_RED[2]);
            } else {
                return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_BLUE[0]) || MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_BLUE[1]) ||
                MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_BLUE[2]);
            }
        }
        return false;
    }
    
    /* entries[0] = forward;
     * entries[1] = side;
     * entries[2] = up;
     * entries[3] = roll;
     * entries[4] = pitch;
     * entries[5] = yaw; */
    public static void setCameraPose(double forward, double up, double pitch) {
        TABLE.getEntry("camerapose_robotspace_set").setDoubleArray(new double[]{forward, 0, up, 0, pitch, 0});

    }

    public static void setPipeline(double idx) {
        TABLE.getEntry("pipeline").setDouble(idx);
    }

    public static double getApriltagId() {
        return TABLE.getEntry("tid").getDouble(0.0);
    }

    public static double getTx() {
        return TABLE.getEntry("tx").getDouble(0.0);
    }

    public static double getTy() {
        return TABLE.getEntry("ty").getDouble(0.0);
    }

    private static double[] getBotPoseVal() {
        return TABLE.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    private static Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

}