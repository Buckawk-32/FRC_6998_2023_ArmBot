package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;


public class AutoBalanceCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final PIDController pidController = new PIDController(Constants.AUTO_BALANCE_KP, Constants.AUTO_BALANCE_KI, Constants.AUTO_BALANCE_KD);
    private final Timer timer = new Timer();
    private Stage current_stage;
    private final boolean WhenFinished_Exit;

    public AutoBalanceCommand(boolean WhenFinished_Exit) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);
        this.current_stage = Stage.Created;
        this.WhenFinished_Exit = WhenFinished_Exit;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        current_stage = Stage.Preparing;
        timer.start();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        double error = -swerveSubsystem.NavX.getPitch();
        switch (current_stage) {
            case Preparing -> {
                swerveSubsystem.Drive(0, Constants.AUTO_BALANCE_START_SPEED_METERS_PER_SEC, 0, false);
                if (Math.abs(error) > Constants.AUTO_BALANCE_TOLERANCE) {
                    current_stage = Stage.Climb;
                    timer.reset();
                }
            }

            case Climb -> {
                swerveSubsystem.Drive(0, pidController.calculate(error), 0, false);
                if (Math.abs(error) <= Constants.AUTO_BALANCE_TOLERANCE) {
                    current_stage = Stage.Wait;
                    timer.reset();
                }
            }

            case Wait -> {
                swerveSubsystem.Drive(0, 0, 0, false);
                if (Math.abs(error) > Constants.AUTO_BALANCE_TOLERANCE) {
                    current_stage = Stage.Climb;
                } else {
                    if (WhenFinished_Exit && timer.get() > Constants.AUTO_BALANCE_WAIT_TIME) {
                        current_stage = Stage.Finish;
                    }
                }
            }
            default ->{
            }
        }
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
        return WhenFinished_Exit && current_stage==Stage.Finish;
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
        swerveSubsystem.Drive(0, 0, 0, false);
        timer.stop();
    }

    public enum Stage {
        Created,
        Preparing,
        Climb,
        Wait,
        Finish
    }
}
