package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    public static final int kLeftMotorID = 9;
    public static final int kRightMotorID = 10;
    // angle of the arm while resting
    public static final Rotation2d kHomeAngle = Rotation2d.fromDegrees(-16.5);
    // shooter_angle == -arm_angle + offset
    public static final Rotation2d kShooterAngleOffset = Rotation2d.fromDegrees(34.5);
    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(95);
    public static final Rotation2d kAngleTolerance = Rotation2d.fromDegrees(1);

    public static final double kStallThresholdAmps = 20;
    public static final double kStallThresholdSeconds = 0.5;

    // (applied to left motor, right motor follows)
    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        feedback.SensorToMechanismRatio = 5 * 4 * (52.0 / 10.0); // 104:1

        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40;

        // SoftwareLimitSwitchConfigs limits = kConfig.SoftwareLimitSwitch;
        // limits.ForwardSoftLimitEnable = true;
        // limits.ForwardSoftLimitThreshold = kMaxAngle.getRotations();
        // limits.ReverseSoftLimitEnable = true;
        // limits.ReverseSoftLimitThreshold = kHomeAngle.getRotations();

        Slot0Configs control = kConfig.Slot0;
        control.kP = 40;
        control.kI = 0;
        control.kD = 0;

        control.GravityType = GravityTypeValue.Arm_Cosine;
        control.kG = 0.25;
        control.kS = 0;
        control.kV = 0;

        MotionMagicConfigs mm = kConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = 2; // rotations per second
        mm.MotionMagicAcceleration = 5; // rotations per second per second
    }
}
