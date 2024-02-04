package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.subsystems.swerve.Drivetrain;

public class Telemetry {
    // private NetworkTable table;
    private NetworkTableInstance inst;

    private NetworkTable _realOutputs;
    private NetworkTable _odometry;

    private NetworkTableEntry rotation;

    private Drivetrain drive = Drivetrain.getInstance();

    private StructArrayPublisher<SwerveModuleState> swerveModuleStates;
    private StructArrayPublisher<SwerveModuleState> swerveModuleStpsOptmized;

    public Telemetry() {
        inst = NetworkTableInstance.getDefault();
        // table = inst.getTable("S");

        _realOutputs = inst.getTable("Real Outputs");
        _odometry = _realOutputs.getSubTable("Odometry");
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