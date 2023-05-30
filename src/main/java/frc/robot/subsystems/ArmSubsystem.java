package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    public CANSparkMax Back_Arm_MOTOR;
    public CANSparkMax Fore_Arm_MOTOR;

    public AbsoluteEncoder Back_Arm_MOTOR_Duty_ENCODER;

    public AbsoluteEncoder Fore_Arm_MOTOR_Duty_ENCODER;

    public CANSparkMax INTAKE_Rotator;

    public CANSparkMax INTAKE_Spin;

    private double angleSetpoint = 0;

    public ArmSubsystem() {

        Back_Arm_MOTOR = new CANSparkMax(Constants.ARM_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        Fore_Arm_MOTOR = new CANSparkMax(Constants.ARM_TOP_MOTOR_ID, MotorType.kBrushless);

        INTAKE_Rotator = new CANSparkMax(Constants.ARM_INTAKE_ROTATOR_MOTOR_ID, MotorType.kBrushless);
        INTAKE_Spin = new CANSparkMax(Constants.ARM_INTAKE_SPIN_MOTOR_ID, MotorType.kBrushless);

        Back_Arm_MOTOR_Duty_ENCODER = Back_Arm_MOTOR.getAbsoluteEncoder(Type.kDutyCycle);
        Fore_Arm_MOTOR_Duty_ENCODER = Fore_Arm_MOTOR.getAbsoluteEncoder(Type.kDutyCycle);

        Back_Arm_MOTOR.restoreFactoryDefaults();
        Fore_Arm_MOTOR.restoreFactoryDefaults();

        Back_Arm_MOTOR.setSmartCurrentLimit(35);
        Fore_Arm_MOTOR.setSmartCurrentLimit(35);

        Fore_Arm_MOTOR.setSoftLimit(SoftLimitDirection.kForward, 130);
        Fore_Arm_MOTOR.enableSoftLimit(SoftLimitDirection.kForward, true);

        Back_Arm_MOTOR.getPIDController().setFF(Constants.ARM_BOTTOM_KF);
        Back_Arm_MOTOR.getPIDController().setP(Constants.ARM_BOTTOM_KP);
        Back_Arm_MOTOR.getPIDController().setSmartMotionMaxVelocity(Constants.ARM_BOTTOM_MAX_VELOCITY_DEG_PER_SEC, 0);
        Back_Arm_MOTOR.getPIDController().setSmartMotionMaxAccel(Constants.ARM_BOTTOM_MAX_ACCELERATION_DEG_PER_SQ, 0);
        Back_Arm_MOTOR.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.ARM_BOTTOM_MAX_ALLOWABLE_ERROR_DEG, 0);

        Fore_Arm_MOTOR.getPIDController().setFF(Constants.ARM_TOP_KF);
        Fore_Arm_MOTOR.getPIDController().setP(Constants.ARM_TOP_KP);
        Fore_Arm_MOTOR.getPIDController().setSmartMotionMaxVelocity(Constants.ARM_TOP_MAX_VELOCITY_DEG_PER_SEC, 0);
        Fore_Arm_MOTOR.getPIDController().setSmartMotionMaxAccel(Constants.ARM_TOP_MAX_ACCELERATION_DEG_PER_SQ, 0);
        Fore_Arm_MOTOR.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.ARM_TOP_MAX_ALLOWABLE_ERROR_DEG, 0);

        Fore_Arm_MOTOR.getEncoder().setPositionConversionFactor(1 / Constants.ARM_TOP_GEARING * 360);
        Fore_Arm_MOTOR.getEncoder().setVelocityConversionFactor(1 / Constants.ARM_TOP_GEARING * 360 / 60);
        Back_Arm_MOTOR.getEncoder().setPositionConversionFactor(1 / Constants.ARM_BOTTOM_GEARING * 360);
        Back_Arm_MOTOR.getEncoder().setVelocityConversionFactor(1 / Constants.ARM_BOTTOM_GEARING * 360 / 60);

        Fore_Arm_MOTOR_Duty_ENCODER.setPositionConversionFactor(360);
        Fore_Arm_MOTOR_Duty_ENCODER.setVelocityConversionFactor(360 / 60.0);
        Fore_Arm_MOTOR_Duty_ENCODER.setZeroOffset(Constants.ARM_TOP_ENCODER_OFFSET % 360);

        Back_Arm_MOTOR_Duty_ENCODER.setPositionConversionFactor(360);
        Back_Arm_MOTOR_Duty_ENCODER.setPositionConversionFactor(360 / 60.0);
        Back_Arm_MOTOR_Duty_ENCODER.setZeroOffset(Constants.ARM_BOTTOM_ENCODER_OFFSET % 360);

        INTAKE_Rotator.setSmartCurrentLimit(Constants.ARM_INTAKE_ROTATOR_MOTOR_CURRENTLIMIT);
        INTAKE_Rotator.getPIDController().setP(Constants.INTAKE_ROTATOR_MOTOR_KP);
        INTAKE_Rotator.getPIDController().setI(Constants.INTAKE_ROTATOR_MOTOR_KI);
        INTAKE_Rotator.getPIDController().setD(Constants.INTAKE_ROTATOR_MOTOR_KD);
        INTAKE_Rotator.getPIDController().setFF(Constants.INTAKE_ROTATOR_MOTOR_KF);

        INTAKE_Spin.setSmartCurrentLimit(Constants.ARM_INTAKE_SPIN_MOTOR_CURRENTLIMIT);
        INTAKE_Spin.getPIDController().setP(Constants.INTAKE_SPIN_MOTOR_KP);
        INTAKE_Spin.getPIDController().setI(Constants.INTAKE_SPIN_MOTOR_KI);
        INTAKE_Spin.getPIDController().setD(Constants.INTAKE_SPIN_MOTOR_KD);
        INTAKE_Spin.getPIDController().setFF(Constants.INTAKE_SPIN_MOTOR_KF);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Fore_Angle", Fore_Arm_MOTOR.getEncoder().getPosition());
        SmartDashboard.putNumber("Back_Angle", Back_Arm_MOTOR.getEncoder().getPosition());
    }

    public void raiseArm() {
        angleSetpoint+=10;
        if (angleSetpoint>130) angleSetpoint = 130;
        Fore_Arm_MOTOR.getPIDController().setReference(angleSetpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    public void lowerArm() {
        angleSetpoint -= 10;
        if (angleSetpoint>130) angleSetpoint = 130;
        Fore_Arm_MOTOR.getPIDController().setReference(angleSetpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    public void zeroBottomArm() {
        Back_Arm_MOTOR.getPIDController().setReference(0, CANSparkMax.ControlType.kSmartMotion);
    }

    public void raiseBottomArm() {
        Back_Arm_MOTOR.getPIDController().setReference(30, CANSparkMax.ControlType.kSmartMotion);
    }

    public void lowerBottomArm() {
        Back_Arm_MOTOR.getPIDController().setReference(-30, CANSparkMax.ControlType.kSmartMotion);
    }

    public void Rotate_Intake(double joy) {
        INTAKE_Rotator.getPIDController().setReference(joy, CANSparkMax.ControlType.kSmartMotion);
    }

    public void Spin_Intake(double spin_rate) {
        INTAKE_Spin.getPIDController().setReference(spin_rate, CANSparkMax.ControlType.kVelocity);
    }

    public void Spin_Back_Intake (double spin_rate) {
        INTAKE_Spin.getPIDController().setReference(-spin_rate, CANSparkMax.ControlType.kVelocity);
    }


}
