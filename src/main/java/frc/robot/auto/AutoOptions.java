package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.intake.Intake;

public class AutoOptions {
    public SendableChooser<Command> autoOptions = new SendableChooser<Command>();
    private CommandSwerveDrivetrain drive;
    private Intake intake;
    private Arm arm;
    private Superstructure superstructure;

    private boolean autosSetup = false;

    public AutoOptions(CommandSwerveDrivetrain drive, Intake intake, Arm arm, Superstructure superstructure) {
        this.drive = drive;
        this.intake = intake;
        this.arm = arm;
        this.superstructure = superstructure;

        SmartDashboard.putData(autoOptions);

        // AutoBuilder.configure(
        //     ()->drive.getState().Pose, 
        //     (resetPose)->drive.resetPose(resetPose), 
        //     ()->drive.getState().Speeds, 
        //     (targetChassisSpeeds)->drive.applyRequest(()->new SwerveRequest.ApplyRobotSpeeds().withSpeeds(targetChassisSpeeds)), 
        //     AutoConstants.kPathFollowingController, 
        //     AutoConstants.kRobotConfig, 
        //     ()->false, 
        //     drive
        // );


        System.out.println(AutoBuilder.isConfigured());
        System.out.println(AutoBuilder.isPathfindingConfigured());
        System.out.println(AutoBuilder.getAllAutoNames());
        addAutoMethods();
    }
    public void robotInit() {
        FollowPathCommand.warmupCommand().schedule();
    }

    // private Command resetInitialOdomC() {
    //     return runOnce(()->{
    //         Rotation2d initialRot = new Rotation2d();
    //         if(drive.getIsDrivingMirrored()){
    //             initialRot = new Rotation2d(Math.PI);
    //         }
    //         drive.resetOdometry(
    //             new Pose2d(
    //                 drive.getPose().getTranslation(),
    //                 initialRot
    //             )
    //         );
    //     });
    // }

    public void periodic(){
        if (!autosSetup && !DriverStation.getAlliance().isEmpty()){
            autoOptions.setDefaultOption("none", drive.runOnce(() -> drive.seedFieldCentric()));
            addAutoOptions();
            autosSetup = true;
        }
    }

    // public Command getAutonomousCommand() {
    //     try{
    //         // Load the path you want to follow using its name in the GUI
    //         PathPlannerPath path = PathPlannerPath.fromPathFile("Bottom Left to Right");
    
    //         // Create a path following command using AutoBuilder. This will also trigger event markers.
    //         return AutoBuilder.followPath(path);
    //     } catch (Exception e) {
    //         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //         return Commands.none();
    //     }
    //   }

    public void addAutoOptions(){
        autoOptions.addOption("Left: Close",
            AutoBuilder.buildAuto("Top Left")
        );
        autoOptions.addOption("Left: Far",
            AutoBuilder.buildAuto("Top Right")
        );
        autoOptions.addOption("Left: Away",
            AutoBuilder.buildAuto("Top Left to Right")
        );
        autoOptions.addOption("Left: Toward",
            AutoBuilder.buildAuto("Top Right to Left")
        );
        autoOptions.addOption("Right: Close",
            AutoBuilder.buildAuto("Bottom Left")
        );
        autoOptions.addOption("Right: Far",
            AutoBuilder.buildAuto("Bottom Right")
        );
        autoOptions.addOption("Right: Away",
            AutoBuilder.buildAuto("Bottom Left to Right")
        );
        autoOptions.addOption("Right: Toward",
            AutoBuilder.buildAuto("Bottom Right to Left")
        );
    }


    public Command getAuto(){
        return Optional.ofNullable(autoOptions.getSelected()).orElse(none());
    }

    private void addAutoMethods(){
        NamedCommands.registerCommand("Intake", superstructure.intake());
        NamedCommands.registerCommand("outtakeTote", superstructure.outtakeTote().withTimeout(1).finallyDo(()->intake.setVoltageC(0)));
        NamedCommands.registerCommand("outtakeBunny", 
            sequence(
                arm.setRotationC(ArmConstants.kToteAngle).withTimeout(0.75),
                intake.setVoltageC(2.5)
            ));
        NamedCommands.registerCommand("intakeOff", run(()->intake.setVoltageC(0)));

    }
}
