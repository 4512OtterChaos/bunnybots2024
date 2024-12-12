package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

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
            arm.setRotationC(ArmConstants.kIntakeAngle),
            intake.setVelocityInC()
        );
    }

    public Command outtakeTote(){
        return sequence(
            arm.setRotationC(ArmConstants.kToteAngle),
            intake.setVoltageOutC()
        );
    }

    public Command decreaseAngle(){
        if (intake.getCurrentCommand() == intake.setVoltageInC()){
            return run(()->arm.decreaseAngle(3)).finallyDo(()->arm.setRotationC(ArmConstants.kIntakeAngle));
        }
        return Commands.none();
    }
}
