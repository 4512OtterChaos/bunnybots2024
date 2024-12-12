package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

        //Update mechanism 2D
        visualizeState(getArmRotations());
        visualizeSetpoint(targetAngle.getRotations());

        log();
    }

    public double getArmRotations() {
        return positionStatus.getValueAsDouble();
    }

    public double getTargetRotations() {
        return targetAngle.getRotations();
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
        isManual = false;
        this.targetAngle = Rotation2d.fromDegrees(MathUtil.clamp(targetAngle.getDegrees() - angleDecrease, kMinAngle.getDegrees(), Units.rotationsToDegrees(getArmRotations())));
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
        if(RobotBase.isSimulation()){ return setRotationC(kHomeAngle);}
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
        SmartDashboard.putData("Arm/Mech2d", mech);
    }

    Mechanism2d mech = new Mechanism2d(.8, .8, new Color8Bit(0, 100, 150));
    MechanismRoot2d mechRoot = mech.getRoot("arm", 0.4, 0.1);

    private final Color8Bit kSetpointBaseColor = new Color8Bit(150, 0, 0);
    private final Color8Bit kSetpointExtensionColor = new Color8Bit(180, 0, 0);
    private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);
    private final Color8Bit kMechExtensionColor = new Color8Bit(0, 0, 180);

    private final double kSetpointWidth = 6;
    private final double kMechWidth = 9;

    private final double kDefaultArmDeg = 90;

    private final MechanismLigament2d mechArm = mechRoot.append(
            new MechanismLigament2d("Arm", kPivotHeight, 90, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechIntake = mechArm.append(
            new MechanismLigament2d("Intake", kiPivotToWheels, kDefaultArmDeg, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechIntakeInside = mechIntake.append(
            new MechanismLigament2d("IntakeInside", Units.inchesToMeters(5.548), 152.4, kMechWidth/2,
                    kMechExtensionColor));
    private final MechanismLigament2d mechJaw = mechIntakeInside.append(
            new MechanismLigament2d("Jaw", Units.inchesToMeters(5.241), -92, kMechWidth/2, kMechExtensionColor));
    private final MechanismLigament2d mechIntakeBottom = mechJaw.append(
            new MechanismLigament2d("IntakeBottom", Units.inchesToMeters(12.508), 151.5, kMechWidth/2, kMechExtensionColor));

    private final MechanismLigament2d setpointArm = mechRoot.append(
            new MechanismLigament2d("setpointArmBase", kPivotHeight, 90, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d setpointIntake = setpointArm.append(
            new MechanismLigament2d("setpointArm", kiPivotToWheels, kDefaultArmDeg, kSetpointWidth,
                    kSetpointBaseColor));
    private final MechanismLigament2d setpointIntakeInside = setpointIntake.append(
            new MechanismLigament2d("setpointIntakeInside", Units.inchesToMeters(5.548), 152.4, kSetpointWidth/2,
                    kSetpointExtensionColor));
    private final MechanismLigament2d setpointJaw = setpointIntakeInside.append(
            new MechanismLigament2d("setpointJaw", Units.inchesToMeters(5.241), -92, kSetpointWidth/2,
                    kSetpointExtensionColor));
    private final MechanismLigament2d setpointIntakeBottom = setpointJaw.append(
            new MechanismLigament2d("IntakeBottom", Units.inchesToMeters(12.508), 151.5, kSetpointWidth/2,
                    kSetpointExtensionColor));
    
    SingleJointedArmSim armSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getFalcon500(2),
            kMomentOfInertia,
            kGearRatio
        ),
        DCMotor.getFalcon500(2),
        kGearRatio,
        kArmLength,
        kMinAngle.getRadians(),
        kHomeAngle.getRadians(),
        true,
        kHomeAngle.getRadians());


    DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getFalcon500(2),
            kMomentOfInertia,
            kGearRatio
        ),
        DCMotor.getFalcon500(2)
    );
    
    public void visualizeState(double armPosRotations) {
        mechIntake.setAngle(90 - Units.rotationsToDegrees(armPosRotations));
    }
    
    public void visualizeSetpoint(double targetPosRotations) {
        setpointIntake.setAngle(90 - Units.rotationsToDegrees(targetPosRotations));
    }
    
    
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = leftMotor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;//TODO: Fix, idk what it means

        motorSimState.setSupplyVoltage(leftMotor.getSupplyVoltage().getValue());//TODO: Add friction? Also, idk that the voltage should be accessed like this

        motorSim.setInputVoltage(motorSimState.getMotorVoltage());

        motorSim.update(0.02);

        motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * 60);//TODO: 60 is gearing
        motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60.0 * 60);//TODO: 60 is gearing
        
        double voltage = motorSim.getInputVoltage();
        armSim.setInput(voltage);
		armSim.update(0.02);
    }
        
}
