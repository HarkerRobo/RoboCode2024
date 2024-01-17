package frc.robot;

import edu.wpi.first.math.util.Units;

public class RobotMap {

    public static final double MAX_VOLTAGE = 10;
    public static final double ROBOT_LOOP = 0.02;
    public static final int MAX_CAN_FRAME_PERIOD = 255;
    
    // Global Robot Constants
    public static final double MAX_DRIVING_SPEED = 4.0; // m/s //TODO
    public static final double MAX_ANGLE_VELOCITY = Math.PI;
    public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY / 2;

    public static final String CAN_CHAIN = "rio";

    // Robot Dimensions
    public static final double ROBOT_LENGTH = Units.inchesToMeters(30); //TODO
    public static final double ROBOT_WIDTH = Units.inchesToMeters(28); //TODO

    public static final class OI {
        public static final double JOYSTICK_DEADBAND = 0.15;
        public static final double TRIGGER_DEADBAND = 0.1;
      
        public static final int DRIVER_ID = 0;
        public static final int OPERATOR_ID = 1;
    }

    public static final class PID {
        public static final int PID_PRIMARY = 0;
        public static final int PID_AUXILIARY = 1;
      
        public static final int SLOT_INDEX = 0;
    }

    public static final class SwerveModule {
        // id of translation motors
        public static final int[] TRANSLATION_IDS = {0, 0, 0, 0};

        // translation motors inverted
        public static final boolean[] TRANSLATION_INVERTS = {false, false, false, true};

        // ids for rotation motors
        public static final int[] ROTATION_IDS = {0, 0, 0, 0};

        // rotation motors inverted
        public static final boolean[] ROTATION_INVERTS = {false, false, false, false};

        // cancoder ids
        public static final int[] CAN_CODER_ID = {0, 0, 0, 0};

        // offsets of cancoders of each swerve module (in rotations)
        public static final double[] CAN_CODER_OFFSETS = {0, 0, 0, 0};

        // current limit constants for translation motors
        public static final double TRANS_CURRENT_LIMIT = 30;
        public static final double TRANS_THRESHOLD_CURRENT = 55;
        public static final double TRANS_THRESHOLD_TIME= 0.1;

        // current limit constants for rotation motors
        public static final double ROT_CURRENT_LIMIT = 25;
        public static final double ROT_THRESHOLD_CURRENT = 40;
        public static final double ROT_THRESHOLD_TIME = 0.1;

        // gear ratios
        public static final double TRANSLATION_GEAR_RATIO = 6.75; //TODO
        public static final double ROTATION_GEAR_RATIO = 12.8; //TODO
        // diameter of the wheel
        public static final double WHEEL_DIAMETER = 4.0; // inches

        // conversions from rotations
        public static final double TRANS_ROT_TO_METERS = WHEEL_DIAMETER * Math.PI / TRANSLATION_GEAR_RATIO; // rotations to meters
        public static final double ROT_ROT_TO_ANGLE = 360.0 / ROTATION_GEAR_RATIO; // rotations to degrees

        // rotation kP
        public static final double ROTATION_KP = 0;

        // Translation FF Values
        public static final double TRANSLATION_KS = 0.02569;
        public static final double TRANSLATION_KV = 1.954584;
        public static final double TRANSLATION_KA = 0.21522;

        // pid
        public static final double TRANSLATION_KP = 0.076;
        public static final double TRANSLATION_KI = 0.00;
        public static final double TRANSLATION_KD = 0.00; 
    }

    public static final class Drivetrain {
        // Pigeon ID
        public static final int PIGEON_ID = 1;

        public static final double PIGEON_kP = 0.067;

        public static final double MIN_OUTPUT = 0.05;

        public static final double MAX_ERROR_YAW = 0.5; //TODO
        public static final double OFFSET = 9.5;

        // Profiled PID for theta (turning) control
        public static final double THETA_P = 0.118;
        public static final double THETA_I = 0.0;
        public static final double THETA_D = 0.0;
    }

    public static final class Limelight {
        public static final double LIMELIGHT_KP = 0; //TODO
        public static final double LIMELIGHT_KI = 0; //TODO
        public static final double LIMELIGHT_KD = 0; //TODO

        public static final int LIMELIGHT_IZONE = 0; //TODO

        public static final double LIMELIGHT_THRESHOLD = 0; //TODO

    }
}