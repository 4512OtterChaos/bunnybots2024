// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.OCXboxController;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final OCXboxController driver = new OCXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();

    public RobotContainer() {
        configureBindings();
        setSwerveUpdateFrequency(drivetrain.getModule(0).getDriveMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(0).getSteerMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(1).getDriveMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(1).getSteerMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(2).getDriveMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(2).getSteerMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(3).getDriveMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(3).getSteerMotor());
        ParentDevice.optimizeBusUtilizationForAll(drivetrain.getModule(0).getDriveMotor(), drivetrain.getModule(0).getSteerMotor(), drivetrain.getModule(1).getDriveMotor(), drivetrain.getModule(1).getSteerMotor(), drivetrain.getModule(2).getDriveMotor(), drivetrain.getModule(3).getDriveMotor(), drivetrain.getModule(3).getSteerMotor());

    }
    
    public void setSwerveUpdateFrequency(TalonFX motor) {
        motor.getDutyCycle().setUpdateFrequency(100);
        motor.getMotorVoltage().setUpdateFrequency(100);
        motor.getPosition().setUpdateFrequency(100);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(50);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getForward() * driver.getDriveSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getStrafe() * driver.getDriveSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getTurn() * driver.getTurnSpeed()) // Drive counterclockwise with negative X (left)
            )
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.leftTrigger().whileTrue(intake.setVoltageInC());
        driver.leftBumper().whileTrue(intake.setVoltageOutC());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
