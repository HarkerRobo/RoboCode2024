package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public final class Limelight {
    private static NetworkTableInstance table;
    public static final String LIMELIGHT_TABLE_KEY = "limelight";

    public static Pose2d getBotPose2d() {
        return toPose2D(getBotPoseVal());
    }

    public static double getBestTargetArea() {
        return getValue("ta").getDouble(0.0);
    }

    public static boolean isPoseValid() {
        return getDistanceToTag() <= RobotMap.Camera.MAX_ERROR_VISION_POSE;
    }

    public static double getDistanceToTag() {
        double[] targetPose = getValue("targetpose_robotspace").getDoubleArray(new double[6]);
        double x = targetPose[0];
        double y = targetPose[1];
        return Math.sqrt((x * x) + (y * y));
    }

    public static boolean isPoseNear(Pose2d pose, Pose2d visionPose) {
        return getDistanceBetweenPose(pose, visionPose) < 1.0
                && MathUtil.compareDouble(visionPose.getTranslation().getNorm(), 0.0);

    }

    public static double getDistanceBetweenPose(Pose2d pose, Pose2d visionPose) {
        return pose.getTranslation().getDistance(visionPose.getTranslation());
    }

    public static double getTimestamp() {
        return Timer.getFPGATimestamp() - getBotPoseVal()[5] / 1000.0;
    }

    public static boolean hasTargets() {
        return getApriltagId() > -1;
    }

    public static boolean atAmp() {
        if (hasTargets()) {
            if (Flip.isFlipped()) {
                return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_AMP_RED);
            } else {
                return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_AMP_BLUE);
            }
        }
        return false;
    }

    public static boolean atSpeaker() {
        if (hasTargets()) {

                if (Flip.isFlipped()) {
                    return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_SPEAKER_RED[0]) || MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_SPEAKER_RED[1]);
                } else {
                    return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_SPEAKER_BLUE[0]) || MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_SPEAKER_BLUE[1]);
                }
        }
        return false;
    }

    public static boolean atStage() {
        if (hasTargets()) {
                if (Flip.isFlipped()) {
                    return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_RED[0]) || MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_RED[1]) || 
                    MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_RED[2]);
                } else {
                    return MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_BLUE[0]) || MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_BLUE[1]) ||
                    MathUtil.compareDouble(getApriltagId(), RobotMap.Camera.ID_STAGE_BLUE[2]);
                }       

        }
        return false;
    }
    

    public static int getNumTargets() {
        return countStringOccurrences(NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry("json").getString(""), "pts");
    }

    public static int countStringOccurrences(String str, String substr) {
        int occ = 0;
        for (int i = 0; i < str.length() - substr.length() + 1; i++) {
            if (str.substring(i, i + substr.length()).equals(substr)) {
                occ++;
            }
        }

        return occ;
    }



    /* entries[0] = forward;
     * entries[1] = side;
     * entries[2] = up;
     * entries[3] = roll;
     * entries[4] = pitch;
     * entries[5] = yaw; */
    public static void setCameraPose(double forward, double up, double pitch) {
        getValue("camerapose_robotspace").setDoubleArray(new double[]{forward, 0, up, 0, pitch, 0});

    }

    public static void setPipeline(double idx) {
        getValue("pipeline").setDouble(idx);
    }

    public static double getApriltagId() {
        return getValue("tid").getDouble(0.0);
    }

    public static double getTx() {
        return getValue("tx").getDouble(0.0);
    }

    public static double getTy() {
        return getValue("ty").getDouble(0.0);
    }

    public static double[] getBotPoseVal() {
        return getValue("botpose_wpiblue").getDoubleArray(new double[7]);
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

    public static NetworkTableEntry getValue(String key) {
        if (table == null) {
            table = NetworkTableInstance.getDefault();
            table.getTable(LIMELIGHT_TABLE_KEY).getEntry("pipeline").setNumber(0);
        }

        return table.getTable(LIMELIGHT_TABLE_KEY).getEntry(key);
    }

}