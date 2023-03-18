package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveDriveCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final XboxController Drive_con;

    public SwerveDriveCommand(XboxController Drive_con) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);

        this.Drive_con = Drive_con;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        double Xval = MathUtil.applyDeadband(-Drive_con.getLeftX(), Constants.CHASSIS_DEADBAND_ZONE);
        double Yval = MathUtil.applyDeadband(-Drive_con.getLeftY(), Constants.CHASSIS_DEADBAND_ZONE);
        double Zrot = MathUtil.applyDeadband(-Drive_con.getRightX(), Constants.CHASSIS_DEADBAND_ZONE);

        Xval *= Constants.CHASSIS_MAX_SPEED_METERS_PER_SEC;
        Yval *= Constants.CHASSIS_MAX_SPEED_METERS_PER_SEC;
        Zrot *= Constants.CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC;

        swerveSubsystem.Drive(Xval, Yval, Zrot, Constants.CHASSIS_ENABLE_FIELD_ORIENTABLE_CONTROL);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.StopAll();
    }
}
