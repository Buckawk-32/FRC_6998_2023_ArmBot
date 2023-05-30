package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends CommandBase {
    private final double spinrate;

    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    private final XboxController Operator_con;

    public IntakeCommand(double spinrate, ArmSubsystem armSubsystem, XboxController Operator_con) {
        addRequirements(this.armSubsystem);

        this.spinrate = spinrate;
        this.Operator_con = Operator_con;
    }

    @Override
    public void execute() {
        double rotate_speed = MathUtil.applyDeadband(-Operator_con.getLeftX(), Constants.INTAKE_ROTATOR_DEADBAND);

        armSubsystem.Rotate_Intake(rotate_speed);

        armSubsystem.Spin_Intake(spinrate);
        armSubsystem.Spin_Back_Intake(spinrate);
    }



}
