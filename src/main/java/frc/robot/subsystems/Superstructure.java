package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;


public class Superstructure {
    private CommandSwerveDrivetrain drive;
    private Intake intake;
    private Arm arm;

    public Superstructure(CommandSwerveDrivetrain drive, Intake intake, Arm arm) {
        this.drive = drive;
        this.intake = intake;
        this.arm = arm;
    }

    public Command intake(){
        return sequence(
            arm.setRotationC(ArmConstants.kIntakeAngle).withTimeout(0.75).unless(()->arm.getTargetRotations() <= ArmConstants.kIntakeAngle.getRotations()),
            intake.setVoltageInC()
        );
    }

    public Command outtakeTote(){
        return sequence(
            arm.setRotationC(ArmConstants.kToteAngle).withTimeout(0.75),
            intake.setVoltageOutC()
        );
    }

    public Command decreaseAngle(){
        return run(()->arm.decreaseAngle(1))
            .onlyIf(()->(arm.getTargetRotations() <= ArmConstants.kIntakeAngle.getRotations()) && (arm.getArmRotations() <= ArmConstants.kIntakeAngle.plus(ArmConstants.kAngleTolerance.times(4)).getRotations()));
            //Only activates if: our target angle < our intake angle AND the error between out target and actual angle is small(ish)
    }
}
