package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RobotMap {

    // Global Robot Constants
    public static final double MAX_VOLTAGE = 12;
    public static final double ROBOT_LOOP = 0.02;
    public static final String CAN_CHAIN = "chiling";
    
    public static final class Field {
        // field dimensions in meters
        public static final double FIELD_LENGTH = 16.54;
        public static final double FIELD_WIDTH = 8.21;
        public static final Field2d FIELD = new Field2d();
        public static final Translation2d SPEAKER = new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));
        // public static final Translation2d AMP = new Translation2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00));
        public static final double DIST_TO_STAGE_APRILTAG = 0;
    }

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
        public static final double TRANSLATION_GEAR_RATIO = 6.12;
        public static final double ROTATION_GEAR_RATIO = 150.0 / 7.0; 
        // diameter of the wheel
        public static final double WHEEL_DIAMETER = 4.0; // inches

        // conversions from rotations
        public static final double TRANS_ROT_TO_METERS = WHEEL_DIAMETER * Math.PI / TRANSLATION_GEAR_RATIO; // rotations to meters
        public static final double ROT_ROT_TO_ANGLE = 360.0 / ROTATION_GEAR_RATIO; // rotations to degrees

        // rotation kP
        public static final double ROTATION_kP = 0; // TODO

        // Translation FF Values
        public static final double TRANSLATION_kS = 0; // TODO
        public static final double TRANSLATION_kV = 0; // TODO
        public static final double TRANSLATION_kA = 0; // TODO

        // pid
        public static final double TRANSLATION_kP = 0; // TODO
        public static final double TRANSLATION_kI = 0.00; // TODO
        public static final double TRANSLATION_kD = 0.00;  // TODO
    }

    public static final class Drivetrain {
        // Pigeon ID
        public static final int PIGEON_ID = 1;

        public static final double PIGEON_kP = 0.067; // TODO

        public static final double MIN_OUTPUT = 0.05;

        // PID for omega (turning) control
        public static final double OMEGA_kP = 0.0; // TODO
        public static final double MAX_ERROR_YAW = 0.5; //TODO

        public static final double VX_AMP_kP = 0;
        public static final double MAX_ERROR_DEG_TX_AMP = 0.5;

        public static final double VX_STAGE_kP = 0;
        public static final double VY_STAGE_kP = 0.0;
        public static final double MAX_ERROR_DEG_TX_STAGE = 0.5;
        public static final double MAX_ERROR_DEG_TY_STAGE = 0.5;
        public static final double VERTICAL_DEG_STAGE = 10;
        
        public static final double MAX_ERROR_VISION_POSE = 1.0; // meters

        // Robot Dimensions
        public static final double ROBOT_LENGTH = Units.inchesToMeters(28);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(28);

        public static final double MAX_DRIVING_SPEED = 5.0; // m/s //TODO
        public static final double MAX_ACCELERATION = MAX_DRIVING_SPEED / 2.0;
        public static final double MAX_ANGLE_VELOCITY = Math.PI;
        public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY / 2.0;

        /**
         * PID values for X, Y, and Rotation (THETA)
         */

        public static double X_kP = 0.0; // TODO
        public static double X_kI = 0.0;
        public static double X_kD = 0.0;

        public static double Y_kP = 0.0; // TODO
        public static double Y_kI = 0.0;
        public static double Y_kD = 0.0;

        public static double THETA_kP = 0.0; // TODO
        public static double THETA_kI = 0.0;
        public static double THETA_kD = 0.0;

        // Speed multipliers
        public static final double SPEED_MULTIPLIER = 1.0; // TODO
        public static final double ROT_MULITPLIER = 1.0; // TODO
        public static final double CLAMP_MULTIPLIER = 0.7;
    }

    public static final class Shooter {
        public static final int MASTER_ID = 0;
        public static final int FOLLOWER_ID = 0; 
        public static final int INDEXER_ID = 0;

        public static final int PROX_SENSOR_ID = 0;

        public static final boolean MASTER_INVERT = false;
        public static final boolean FOLLOWER_INVERT = false;
        public static final boolean INDEXER_INVERT = false;

        public static final int SHOOTER_CURRENT_LIMIT = 60;
        public static final int INDEXER_CURRENT_LIMIT = 20;

        public static final double INDEXING_SPEED = 0.1;

        public static final double REV_TIME = 1.0; // seconds

        public static enum Goal {
            AMP,
            SPEAKER,
        }
    }

    public static final class Pivot {
        public static final int MASTER_ID = 0;
        public static final int FOLLOWER_ID = 0; 
        public static final int LIMIT_SWITCH_ID = 0;

        public static final boolean MASTER_INVERT = false;
        public static final boolean FOLLOWER_INVERT = false; 
        
        public static final double ZERO_SPEED = -0.3;

        public static final double PIVOT_kP = 0;
        public static final double PIVOT_kG = 0;
        public static final double PIVOT_kS = 0;
        public static final double PIVOT_kV = 0;
        public static final double PIVOT_kA = 0;

        public static final double TRAP_ANGLE = 30;
        public static final double AMP_ANGLE = 20;

        public static final double PIVOT_GEAR_RATIO = 0;

        public static final double MAX_ERROR = 1; // degrees
    
        public static final double PIVOT_ROT_TO_ANGLE = 360.0 / PIVOT_GEAR_RATIO; // rotations to degrees

        public static final double PIVOT_FORWARD_SOFT_LIMIT = AMP_ANGLE / PIVOT_ROT_TO_ANGLE;
        public static final double PIVOT_REVERSE_SOFT_LIMIT = 0;

        public static final double MAX_CRUISE_ACCLERATION = 0;
        public static final double MAX_CRUISE_VElOCITY = 0;

        public static enum Goal {
            AMP,
            SPEAKER,
            TRAP
        }

    }

    public static final class Elevator {
        public static final int MASTER_ID = 0;
        public static final int FOLLOWER_ID = 0; 
        public static final int LIMIT_SWITCH_ID = 0; 

        public static final double ELEVATOR_kP = 0;
        public static final double ELEVATOR_kG = 0;

        public static final double TRAP_HEIGHT = 0; // motor rotations
        public static final double STAGE_HEIGHT = 0;

        public static final double ELEVATOR_FORWARD_SOFT_LIMIT = 0;
        public static final double ELEVATOR_REVERSE_SOFT_LIMIT = 0;

        public static final boolean MASTER_INVERT = false;
        public static final boolean FOLLOWER_INVERT = false; 

        public static final double ZERO_SPEED = -0.3;

        public static final double MAX_ERROR = 1; // motor rotations
    }

    public static final class Intake {
        public static final int DEPLOY_ID = 0;
        public static final int ROLLER_ID = 0;
        public static final int LIMIT_SWITCH_ID = 0;

        public static final boolean DEPLOY_INVERT = false;
        public static final boolean ROLLER_INVERT = false;

        public static final double ZERO_SPEED = -0.3;
        public static final double ROLLER_SPEED = 0.7;

        public static final double DEPLOY_kP = 0;
        public static final double INTAKE_DEPLOY = 43; // rotations

        public static final int ROLLER_CURRENT_LIMIT = 20;
    }

    public static final class Indexer {
        public static final int MASTER_ID = 0;

        public static final boolean MASTER_INVERT = false;

        public static final double INDEXING_SPEED = 0.5;

        public static final int CURRENT_LIMIT = 10;

    }

    public static final class Camera {
        public static final double FORWARD = Units.inchesToMeters(14); // TODO
        public static final double UP = Units.inchesToMeters(3.75); // meters
        public static final double PITCH = 65; // degrees
    }

}