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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.OCXboxController;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

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
    public final Arm arm = new Arm();
    public final Superstructure superstructure = new Superstructure(drivetrain, intake, arm);


    public RobotContainer() {
        configureBindings();
        intake.setDefaultCommand(intake.setVoltageC(1));
        setSwerveUpdateFrequency(drivetrain.getModule(0).getDriveMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(0).getSteerMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(1).getDriveMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(1).getSteerMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(2).getDriveMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(2).getSteerMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(3).getDriveMotor());
        setSwerveUpdateFrequency(drivetrain.getModule(3).getSteerMotor());
        ParentDevice.optimizeBusUtilizationForAll(drivetrain.getModule(0).getDriveMotor(), drivetrain.getModule(0).getSteerMotor(), drivetrain.getModule(1).getDriveMotor(), drivetrain.getModule(1).getSteerMotor(), drivetrain.getModule(2).getDriveMotor(), drivetrain.getModule(2).getSteerMotor(), drivetrain.getModule(3).getDriveMotor(), drivetrain.getModule(3).getSteerMotor());

        drivetrain.setOperatorPerspectiveForward(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg);
    }
    
    public void setSwerveUpdateFrequency(TalonFX motor) {
        motor.getDutyCycle().setUpdateFrequency(100);
        motor.getMotorVoltage().setUpdateFrequency(100);
        motor.getPosition().setUpdateFrequency(100);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(50);
    }

    private void configureBindings() {
        //DRIVE COMMAND
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getForward() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driver.getStrafe() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(driver.getTurn() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        //INTAKE COMMANDS
        driver.rightTrigger().whileTrue(superstructure.intake());
        driver.rightBumper().whileTrue(superstructure.outtakeTote());
        driver.back().whileTrue(intake.setVoltageOutC());

        //Decrease arm angle (relatively) slowly while intaking
        driver.leftTrigger()
            .whileTrue(superstructure.decreaseAngle());

        driver.povDown().onTrue(arm.homingSequenceC());

        // reset the robot heading to forward
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
