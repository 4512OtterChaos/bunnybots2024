package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    private SparkMax motor = new SparkMax(kFloorMotorID, MotorType.kBrushless);

    RelativeEncoder encoder = motor.getEncoder();
    
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.25, 0.005);
    private PIDController PID = new PIDController(0.005, 0, 0);

    boolean isManual = true;
    
    double lastFreeTime = Timer.getFPGATimestamp();
    
    public Intake(){
        // intakeMotor.setCANTimeout(100);
        // OCConfig.configMotors(kMotorStallLimit, kMotorStallLimit, kRampRate, leftMotor, rightMotor);
        // OCConfig.setStatusNormal(leftMotor, rightMotor);
        // OCConfig.setIdleMode(IdleMode.kBrake, leftMotor, rightMotor);
        // intakeMotor.setCANTimeout(0);
		// OCConfig.saveConfig(leftMotor, rightMotor);
    }

    // @Override
    public void periodic() {
        if(!isManual) {
            motor.setVoltage(ff.calculate(PID.getSetpoint())+PID.calculate(encoder.getVelocity()));
        }

        if (getFloorCurrent() <= kStallCurrent){
            lastFreeTime = Timer.getFPGATimestamp();
        }

        log();
    }

    public void setVoltage(double voltage){
        isManual=true;
        motor.setVoltage(voltage);
    }

    public void setVelocity(double floorRPM) {
        isManual = false;
        PID.setSetpoint(floorRPM);
    }

    public boolean isStalled(){
        return Timer.getFPGATimestamp() >= (lastFreeTime + kStallTime);
    }

    public double getFloorCurrent(){
        return motor.getOutputCurrent();
    }

    public Command setVoltageC(double voltage){
        return run(()->setVoltage(voltage));
    }

    public Command setVoltageInC(){
        return run(()->setVoltage(7));
    }

    public Command setVoltageOutC(){
        return run(()->setVoltage(-7));
    }

    public Command setVelocityC(double RPM){
        return run(()->setVelocity(RPM));
    }

    public Command setVelocityInC(){
        return run(()->setVelocity(30));
    }

    public Command setVelocityOutC(){
        return run(()->setVelocity(-30));
    }

    public void log() {
        SmartDashboard.putNumber("Intake/Motor Voltage", motor.getAppliedOutput()*12);
        SmartDashboard.putNumber("Intake/Motor Current", getFloorCurrent());
        SmartDashboard.putBoolean("Intake/isStalled", isStalled());
    }
}
