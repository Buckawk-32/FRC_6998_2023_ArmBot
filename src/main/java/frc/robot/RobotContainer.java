// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    public final double spinrate = 0;

    public final XboxController Driver_controller = new XboxController(0);
    public final XboxController Operator_Controller = new XboxController(1);

    public ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    public SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

    public PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", new PathConstraints(Constants.CHASSIS_MAX_SPEED_METERS_PER_SEC, Constants.CHASSIS_MAX_ACCEL_METERS_PER_SEC));
    private final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::GetPose,
            swerveSubsystem::ResetPose,
            swerveSubsystem.SWERVE_DRIVE_KINEMATICS,
            new PIDConstants(1, 0, 0),
            new PIDConstants(0.1, 0, 0),
            swerveSubsystem::setModulesStates,
            null,
            true,
            swerveSubsystem
    );
    // The robot's subsystems and commands are defined here...
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        SwerveDriveCommand swerveDriveCommand = new SwerveDriveCommand(Driver_controller);

        swerveSubsystem.setDefaultCommand(swerveDriveCommand);

        armSubsystem.setDefaultCommand(new IntakeCommand(
                spinrate,
                armSubsystem,
                Operator_Controller
        ));



        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new JoystickButton(Driver_controller, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(swerveSubsystem::ResetGyro));

        new JoystickButton(Operator_Controller, XboxController.Button.kB.value).onTrue(new InstantCommand(armSubsystem::raiseArm));
        new JoystickButton(Operator_Controller, XboxController.Button.kA.value).onTrue(new InstantCommand(armSubsystem::lowerArm));
        new JoystickButton(Operator_Controller, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(armSubsystem::raiseBottomArm));
        new JoystickButton(Operator_Controller, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(armSubsystem::lowerBottomArm));
        new JoystickButton(Operator_Controller, XboxController.Button.kStart.value).onTrue(new InstantCommand(armSubsystem::zeroBottomArm));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return swerveAutoBuilder.fullAuto(trajectory).andThen(new AutoBalanceCommand(false));
    }
}
