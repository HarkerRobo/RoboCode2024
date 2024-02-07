package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.OI;
import frc.robot.subsystems.swerve.Drivetrain;

public class Telemetry {
    // private NetworkTable table;
    private NetworkTableInstance inst;

    private NetworkTable _realOutputs;
    private NetworkTable _odometry;

    private NetworkTable _vision;
    private NetworkTable _targets;
    private NetworkTable _limelight;

    private NetworkTable _debug;

    private NetworkTableEntry rotation;

    private Drivetrain drive = Drivetrain.getInstance();

    private StructArrayPublisher<SwerveModuleState> swerveModuleStates;
    private StructArrayPublisher<SwerveModuleState> swerveModuleStpsOptmized;

    public Telemetry() {
        inst = NetworkTableInstance.getDefault();
        // table = inst.getTable("S");

        _realOutputs = inst.getTable("Real Outputs");
        _odometry = _realOutputs.getSubTable("Odometry");

        _vision = inst.getTable("Vision");
        _targets = _vision.getSubTable("At Targets");
        _limelight = _vision.getSubTable("Limelight");

        _debug = inst.getTable("Debug");
    }

    public void debug() {
        NetworkTableEntry leftTrigger = _debug.getEntry("Left Trigger");
        leftTrigger.setDouble(OI.getInstance().getDriver().getLeftTrigger());

        NetworkTableEntry a = _debug.getEntry("Button A");
        a.setBoolean(OI.getInstance().getDriver().getButtonAState());
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

    public void swerve() {
        // table = inst.getTable("Swerve");
        // NetworkTable _states = table.getSubTable("_states");

        swerveModuleStates = _realOutputs.getSubTable("SwerveModuleStates")
            .getStructArrayTopic("Measured", SwerveModuleState.struct).publish();

        swerveModuleStpsOptmized = _realOutputs.getSubTable("SwerveModuleStates")
            .getStructArrayTopic("Setpoints Optimized", SwerveModuleState.struct).publish();

        rotation = _odometry.getEntry("Rotation");
    }

    public void publish() {
        swerveModuleStates.set(drive.getModuleStates());
        swerveModuleStpsOptmized.set(drive.getOptimizedStates());

        rotation.setDouble(drive.getRotation().getRadians());
    }
}