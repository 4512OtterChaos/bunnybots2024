package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private Rotation2d targetAngle = kHomeAngle;
    private boolean isManual = false;
    private double targetVoltage = 0;

    private double lastNonStallTime = Timer.getFPGATimestamp();

    private boolean isHoming = false;

    private final StatusSignal<Double> dutyStatus = leftMotor.getDutyCycle();
    private final StatusSignal<Voltage> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Angle> positionStatus = leftMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();

    public Arm() {
        // try applying motor configs
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = leftMotor.getConfigurator().apply(kConfig);
            rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
            if (status.isOK()) break;
        }
        if (!status.isOK()) DriverStation.reportWarning("Failed applying Arm motor configuration!", false);

        dutyStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(50);
        statorStatus.setUpdateFrequency(50);
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);

        SmartDashboard.putData("Arm/Subsystem", this);

        resetArmRotations(kHomeAngle.getRotations());
    }

    @Override
    public void periodic() {
        // Angle safety
        double currentRot = getArmRotations();
        double currentKG = Math.cos(getArmRotations()) * kConfig.Slot0.kG;
        double adjustedVoltage = targetVoltage + currentKG;

        if (currentRot <= kMinAngle.getRotations()) {
            adjustedVoltage = Math.min(currentKG, adjustedVoltage);//TODO: Switch max & min?
        }
        if (!isHoming && currentRot >= kHomeAngle.plus(kAngleTolerance).getRotations() && targetAngle.getRotations() >= kHomeAngle.plus(kAngleTolerance.times(3)).getRotations()) {
            adjustedVoltage = Math.max(adjustedVoltage, 0);//TODO: Switch max & min?
            if (!isManual) { // go limp at home angle
                isManual = true;
                adjustedVoltage = 0;
            }
        }

        // Voltage/Position control
        if (!isManual) {
            leftMotor.setControl(mmRequest.withPosition(targetAngle.getRotations()));
        }
        else {
            leftMotor.setControl(voltageRequest.withOutput(adjustedVoltage));
        }

        // Stall detection
        if (getCurrent() < kStallThresholdAmps) {
            lastNonStallTime = Timer.getFPGATimestamp();
        }

        log();
    }

    public double getArmRotations() {
        return positionStatus.getValueAsDouble();
    }

    public boolean isWithinTolerance() {
        double error = targetAngle.minus(Rotation2d.fromRotations(getArmRotations())).getRotations();
        return Math.abs(error) < kAngleTolerance.getRotations();
    }

    public void resetArmRotations(double rotations){
        leftMotor.setPosition(rotations);
    }

    public void setVoltage(double volts){
        isManual = true;
        targetVoltage = volts;
    }

    public void setRotation(Rotation2d targetRot){
        isManual = false;
        this.targetAngle = Rotation2d.fromRotations(MathUtil.clamp(targetRot.getRotations(), kMinAngle.getRotations(), kHomeAngle.getRotations()));
    }

    public void decreaseAngle(double angleDecrease){//TODO: Change isManual?
        this.targetAngle = Rotation2d.fromDegrees(MathUtil.clamp(targetAngle.getDegrees() - angleDecrease, kMinAngle.getDegrees(), kHomeAngle.getDegrees()));
    }

    public void stop(){ 
        setVoltage(0);
    }

    public double getVelocity(){
        return velocityStatus.getValueAsDouble();
    }

    public double getCurrent(){
        return statorStatus.getValueAsDouble();
    }

    public boolean isStalled(){
        return (Timer.getFPGATimestamp() - lastNonStallTime) > kStallThresholdSeconds;
    }

    //---------- Command factories

    /** Sets the arm voltage and ends immediately. */
    public Command setVoltageC(double volts) {
        return runOnce(()->setVoltage(volts));
    }

    /** Sets the target arm rotation and ends when it is within tolerance. */
    public Command setRotationC(Rotation2d targetRot){
        return run(()->setRotation(targetRot)).until(this::isWithinTolerance);
    }

    /** Runs the arm into the hardstop, detecting a current spike and resetting the arm angle. */
    public Command homingSequenceC(){
        return startEnd(
            () -> {
                isHoming = true;
                setVoltage(2);
            },
            () -> {
                isHoming = false;
                setVoltage(0);
                resetArmRotations(kHomeAngle.getRotations());
            }
        ).until(()->isStalled());
    }

    public void log() {
        SmartDashboard.putNumber("Arm/Arm Native", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Arm Degrees", Units.rotationsToDegrees(getArmRotations()));
        SmartDashboard.putNumber("Arm/Arm Target Degrees", targetAngle.getDegrees());
        SmartDashboard.putNumber("Arm/Motor Current", getCurrent());
        SmartDashboard.putBoolean("Arm/isStalled", isStalled());
        SmartDashboard.putNumber("Arm/Motor Voltage", voltageStatus.getValueAsDouble());
        SmartDashboard.putNumber("Arm/Motor Target Voltage", targetVoltage);
        SmartDashboard.putNumber("Arm/Motor Velocity", getVelocity());
        SmartDashboard.putBoolean("Arm/Motor Stalled", isStalled());
        SmartDashboard.putNumber("Arm/Time", Timer.getFPGATimestamp());
    }

    SingleJointedArmSim armSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(
            kConfig.Slot0.kV,
            kConfig.Slot0.kA
        ),
        DCMotor.getFalcon500(2),
        60,
        Units.inchesToMeters(14.8),
        kMinAngle.getRadians(),
        kHomeAngle.getRadians(),
        true,
        kHomeAngle.getRadians());

    DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.identifyPositionSystem(
            kConfig.Slot0.kV,
            kConfig.Slot0.kA
        ),
        DCMotor.getFalcon500(2));

    PWMSim testSim = new PWMSim(8);
    
    public void simulationPeriodic() {
        double voltage = motorSim.getInputVoltage();
        armSim.setInput(voltage);
		armSim.update(0.02);
    }
}
