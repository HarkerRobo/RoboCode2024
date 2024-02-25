package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;

public class Telemetry {
    // private NetworkTable table;
    private NetworkTableInstance inst;

    private static NetworkTable _realOutputs;
    private static NetworkTable _swerve;

    private NetworkTable _odometry;
    private NetworkTable _autons;

    private NetworkTable _modules;
    private static NetworkTable _zero;
    private static NetworkTable _one;
    private static NetworkTable _two;
    private static NetworkTable _three;

    private NetworkTable _vision;
    private NetworkTable _targets;
    private NetworkTable _limelight;

    private NetworkTable _debug;
    private NetworkTable _controls;

    private Drivetrain drive = Drivetrain.getInstance();
    private XboxGamepad oiDriver = OI.getInstance().getDriver();

    private StructArrayPublisher<SwerveModuleState> swerveModuleStates;
    private StructArrayPublisher<SwerveModuleState> swerveModuleStpsOptmized;

    public Telemetry() {
        inst = NetworkTableInstance.getDefault();
        // table = inst.getTable("S");

        _realOutputs = inst.getTable("Real Outputs");
        
        _swerve = inst.getTable("Swerve");
        
        _odometry = _realOutputs.getSubTable("Odometry");
        _autons = _realOutputs.getSubTable("Autons");

        _modules = _swerve.getSubTable("Modules");
        _zero = _modules.getSubTable("0");
        _one = _modules.getSubTable("1");
        _two = _modules.getSubTable("2");
        _three = _modules.getSubTable("3");

        _vision = inst.getTable("Vision");
        _targets = _vision.getSubTable("At Targets");
        _limelight = _vision.getSubTable("Limelight");

        _debug = _realOutputs.getSubTable("Debug");
        _controls = _debug.getSubTable("Controls");
    }

    public void autons(String entry, String list) {
        NetworkTableEntry cAuton = _autons.getEntry(entry);
        cAuton.setString(list);
    }

    public void odometry() {
        NetworkTableEntry rotation = _odometry.getEntry("Rotation");
        rotation.setDouble(drive.getRotation().getRadians());

        NetworkTableEntry distance = _odometry.getEntry("Distance");
        distance.setDouble(drive.getDistanceToSpeaker());
    }

    public void debug() {
        NetworkTableEntry a = _controls.getEntry("Button A");
        a.setBoolean(oiDriver.getButtonAState());

        NetworkTableEntry b = _controls.getEntry("Button B");
        b.setBoolean(oiDriver.getButtonBState());

        NetworkTableEntry x = _controls.getEntry("Button X");
        x.setBoolean(oiDriver.getButtonXState());

        NetworkTableEntry y = _controls.getEntry("Button Y");
        y.setBoolean(oiDriver.getButtonYState());

        NetworkTableEntry leftBumper = _controls.getEntry("Left Bumper");
        leftBumper.setBoolean(oiDriver.getLeftBumperState());

        NetworkTableEntry rightBumper = _controls.getEntry("Right Bumper");
        rightBumper.setBoolean(oiDriver.getRightBumperState());

        NetworkTableEntry leftTrigger = _controls.getEntry("Left Trigger");
        leftTrigger.setDouble(oiDriver.getLeftTrigger());

        NetworkTableEntry rightTrigger = _controls.getEntry("Right Trigger");
        rightTrigger.setDouble(oiDriver.getRightTrigger());

        NetworkTableEntry upD = _controls.getEntry("Up DPad");
        upD.setBoolean(oiDriver.getUpDPadButtonState());

        NetworkTableEntry leftD = _controls.getEntry("Left DPad");
        leftD.setBoolean(oiDriver.getLeftDPadButtonState());

        NetworkTableEntry downD = _controls.getEntry("Down DPad");
        downD.setBoolean(oiDriver.getDownDPadButtonState());

        NetworkTableEntry rightD = _controls.getEntry("Right DPad");
        rightD.setBoolean(oiDriver.getDownDPadButtonState());

        NetworkTableEntry select = _controls.getEntry("Select");
        select.setBoolean(oiDriver.getButtonSelectState());

        NetworkTableEntry start = _controls.getEntry("Start");
        start.setBoolean(oiDriver.getButtonStartState());


        NetworkTableEntry isRobotCentric = _debug.getEntry("Robot Centric");
        isRobotCentric.setBoolean(drive.robotCentric());

        NetworkTableEntry intakeLimitSwitchHit = _debug.getEntry("Intake Limit Switch Hit");
        intakeLimitSwitchHit.setBoolean(Intake.getInstance().limitSwitchHit());

        NetworkTableEntry pivotLimitSwitchHit = _debug.getEntry("Pivot Limit Switch Hit");
        pivotLimitSwitchHit.setBoolean(Pivot.getInstance().isLimitHit());

        NetworkTableEntry shooterIndexProxSensor = _debug.getEntry("Shooter Index Occupied");
        shooterIndexProxSensor.setBoolean(Shooter.getInstance().shooterIndexerOccupied());

        NetworkTableEntry elevatorLimitSwitchHit = _debug.getEntry("Elevator Limit Switch Hit");
        elevatorLimitSwitchHit.setBoolean(Elevator.getInstance().isLimitHit());

        NetworkTableEntry elevatorSensorPosition = _debug.getEntry("Elevator Sensor Position");
        elevatorSensorPosition.setDouble(Elevator.getInstance().getPosition());

        NetworkTableEntry pivotSensorPosition = _debug.getEntry("Pivot Sensor Position");
        pivotSensorPosition.setDouble(Pivot.getInstance().getPosition());

        NetworkTableEntry pivotVelocity = _debug.getEntry("Pivot Velocity");
        pivotVelocity.setDouble(Pivot.getInstance().getVelocity());
    }

    public void vision() {
        NetworkTableEntry hasTargets = _targets.getEntry("Has Targets");
        hasTargets.setBoolean(Limelight.hasTargets());

        NetworkTableEntry atAmp = _targets.getEntry("At Amp");
        atAmp.setBoolean(Limelight.atAmp());

        NetworkTableEntry atSpeaker = _targets.getEntry("At Speaker");
        atSpeaker.setBoolean(Limelight.atSpeaker());

        NetworkTableEntry tagId = _limelight.getEntry("Apriltag ID");
        tagId.setDouble(Limelight.getApriltagId());

        NetworkTableEntry tX = _limelight.getEntry("Tx");
        tX.setDouble(Limelight.getTx());

        NetworkTableEntry tY = _limelight.getEntry("Ty");
        tY.setDouble(Limelight.getTy());
    }

    public static void putModule(int id, String entry, double number) {
        switch (id) {
            case 0:
                NetworkTableEntry zeroName = _zero.getEntry("Module Name");
                zeroName.setString("Top Left");

                NetworkTableEntry _entry0 = _zero.getEntry(entry);
                _entry0.setDouble(number);
                break;
            
            case 1:
                NetworkTableEntry oneName = _one.getEntry("Module Name");
                oneName.setString("Top Right");

                NetworkTableEntry _entry1 = _one.getEntry(entry);
                _entry1.setDouble(number);
                break;

            case 2:
                NetworkTableEntry twoName = _two.getEntry("Module Name");
                twoName.setString("Bottom Left");

                NetworkTableEntry _entry2 = _two.getEntry(entry);
                _entry2.setDouble(number);
                break;

            case 3:
                NetworkTableEntry threeName = _three.getEntry("Module Name");
                threeName.setString("Bottom Right");

                NetworkTableEntry _entry3 = _three.getEntry(entry);
                _entry3.setDouble(number);
                break;
        }
    }

    public static void putNumber(String system, String entry, double number) {
        switch (system) {
            case "swerve":
                NetworkTableEntry _entry = _swerve.getEntry(entry);
                _entry.setDouble(number);
                break;
        
            default:
                break;
        }
    }

    // copy above method for every type (string, boolean, etc)

    public void swerveStates() {
        // table = inst.getTable("Swerve");
        // NetworkTable _states = table.getSubTable("_states");

        swerveModuleStates = _realOutputs.getSubTable("SwerveModuleStates")
            .getStructArrayTopic("Measured", SwerveModuleState.struct).publish();

        swerveModuleStpsOptmized = _realOutputs.getSubTable("SwerveModuleStates")
            .getStructArrayTopic("Setpoints Optimized", SwerveModuleState.struct).publish();
    }

    public void publish() {
        swerveModuleStates.set(drive.getModuleStates());
        swerveModuleStpsOptmized.set(drive.getOptimizedStates());
    }
}