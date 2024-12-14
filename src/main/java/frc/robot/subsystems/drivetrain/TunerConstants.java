package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {

    public static final double kMomentOfInertia = 8; //kg*m^2
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    public static final double kTrackLength = Units.inchesToMeters(18.5);
    
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(25).withKI(0).withKD(1.0)
        .withKS(0.1).withKV(1.59).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // private static final Slot0Configs steerGains = new Slot0Configs()
    //     .withKP(100).withKI(0).withKD(2.0)
    //     .withKS(0.2).withKV(1.59).withKA(0)
    //     .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);





    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(2);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(80.0);

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.05);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.122448979591837;
    private static final double kSteerGearRatio = 12.8;
    public static final Distance kWheelRadius = Inches.of(1.93575);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final double kSteerInertia = 0.01;
    private static final double kDriveInertia = 0.01;
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withCANcoderInitialConfigs(cancoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 4;
    private static final int kFrontLeftSteerMotorId = 8;
    private static final int kFrontLeftEncoderId = 2;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.312744140625);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftCANcoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10.25);
    private static final Distance kFrontLeftYPos = Inches.of(9.25);

    // Front Right
    private static final int kFrontRightDriveMotorId = 7;
    private static final int kFrontRightSteerMotorId = 3;
    private static final int kFrontRightEncoderId = 3;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.185302734375);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightCANcoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10.25);
    private static final Distance kFrontRightYPos = Inches.of(-9.25);

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 5;
    private static final int kBackLeftEncoderId = 4;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.21240234375);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftCANcoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10.25);
    private static final Distance kBackLeftYPos = Inches.of(9.25);

    // Back Right
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 6;
    private static final int kBackRightEncoderId = 1;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.46875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightCANcoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10.25);
    private static final Distance kBackRightYPos = Inches.of(-9.25);


    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftCANcoderInverted);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightCANcoderInverted);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftCANcoderInverted);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightCANcoderInverted);

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }
}
