package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;


public class SwerveSubsystem extends SubsystemBase {

    private final static SwerveSubsystem INSTANCE = new SwerveSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static SwerveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Front Left
     * Front Right
     * Rear Left
     * Rear Right
     */
    public final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(Constants.CHASSIS_LENGTH_METERS / 2.0, Constants.CHASSIS_WIDTH_METERS / 2.0),
            new Translation2d(Constants.CHASSIS_LENGTH_METERS / 2.0, -Constants.CHASSIS_WIDTH_METERS / 2.0),
            new Translation2d(-Constants.CHASSIS_LENGTH_METERS / 2.0, Constants.CHASSIS_WIDTH_METERS / 2.0),
            new Translation2d(-Constants.CHASSIS_LENGTH_METERS / 2.0, -Constants.CHASSIS_WIDTH_METERS / 2.0)
    );

    public SwerveModule[] SwerveModules = new SwerveModule[]{

            // Front Left
            new SwerveModule(
                    Constants.CHASSIS_FRONT_LEFT_DRIVE_MOTOR_ID,
                    Constants.CHASSIS_FRONT_LEFT_ANGLE_MOTOR_ID,
                    Constants.CHASSIS_FRONT_LEFT_CANCODER_ID,
                    Constants.CHASSIS_FRONT_LEFT_ANGLE_OFFSET),

            // Back Left
            new SwerveModule(
                    Constants.CHASSIS_BACK_LEFT_DRIVE_MOTOR_ID,
                    Constants.CHASSIS_BACK_LEFT_ANGLE_MOTOR_ID,
                    Constants.CHASSIS_BACK_LEFT_CANCODER_ID,
                    Constants.CHASSIS_BACK_LEFT_ANGLE_OFFSET),

            // Back Right
            new SwerveModule(
                    Constants.CHASSIS_BACK_RIGHT_DRIVE_MOTOR_ID,
                    Constants.CHASSIS_BACK_RIGHT_ANGLE_MOTOR_ID,
                    Constants.CHASSIS_BACK_RIGHT_CANCODER_ID,
                    Constants.CHASSIS_BACK_RIGHT_ANGLE_OFFSET),

            // Front Right
            new SwerveModule(
                    Constants.CHASSIS_FRONT_RIGHT_DRIVE_MOTOR_ID,
                    Constants.CHASSIS_FRONT_RIGHT_ANGLE_MOTOR_ID,
                    Constants.CHASSIS_FRONT_RIGHT_CANCODER_ID,
                    Constants.CHASSIS_FRONT_RIGHT_ANGLE_OFFSET),
    };



    public final AHRS NavX = new AHRS(SPI.Port.kMXP);

    public SwerveDriveOdometry SWERVE_ODOMETRY;

    private SwerveSubsystem() {
        NavX.zeroYaw();
        SWERVE_ODOMETRY = new SwerveDriveOdometry(
                SWERVE_DRIVE_KINEMATICS,
                NavX.getRotation2d(),
                getModulePosition(),
                Constants.CHASSIS_INITIAL_POSE);
    }

    public void ResetGyro() {
        NavX.reset();
    }

    public void ResetPosition(Pose2d pose2d) {
        SWERVE_ODOMETRY.resetPosition(NavX.getRotation2d(), getModulePosition(), pose2d);
    }

    public Pose2d GetPose() {
        return SWERVE_ODOMETRY.getPoseMeters();
    }

    public SwerveDriveKinematics GetKinematics() {
        return SWERVE_DRIVE_KINEMATICS;
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
                SwerveModules[0].getPosition(),
                SwerveModules[1].getPosition(),
                SwerveModules[2].getPosition(),
                SwerveModules[3].getPosition()
        };

    }

    public void Drive(double Xspeed, double Yspeed, double Rot, boolean FieldRelative) {
        SwerveModuleState[] swerveModuleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                FieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, Rot, NavX.getRotation2d()) : new ChassisSpeeds(Xspeed, Yspeed, Rot));
        setModulesStates(swerveModuleStates);
    }

    public void setModulesStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < 4; i++) {
            SwerveModules[i].setDesiredDState(swerveModuleStates[i]);
        }
    }

    public void syncPosition() {
        for (SwerveModule swerveModule : SwerveModules){
            swerveModule.syncPosition();
        }
    }

    public void StopAll() {
        for (SwerveModule swerveModule : SwerveModules) {
            swerveModule.stop();
        }
    }

    @Override
    public void periodic() {
        SWERVE_ODOMETRY.update(NavX.getRotation2d(), getModulePosition());
        SmartDashboard.putNumber("ODOMETRY X", SWERVE_ODOMETRY.getPoseMeters().getX());
        SmartDashboard.putNumber("ODOMETRY Y", SWERVE_ODOMETRY.getPoseMeters().getY());
        SmartDashboard.putNumber("ODOMETRY ANGLE", SWERVE_ODOMETRY.getPoseMeters().getRotation().getDegrees());
    }

    public static class SwerveModule {
        private final TalonFX DRIVE_MOTOR;
        private final CANSparkMax ANGLE_MOTOR;
        private final CANCoder CANCODER;
        private final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(
                Constants.CHASSIS_DRIVE_MOTOR_KS,
                Constants.CHASSIS_DRIVE_MOTOR_KV,
                Constants.CHASSIS_DRIVE_MOTOR_KA
        );

        public SwerveModule(int DRIVE_MOTOR_ID, int ANGLE_MOTOR_ID, int CANCODER_ID, double ANGLE_OFFSET) {
            DRIVE_MOTOR = new WPI_TalonFX(DRIVE_MOTOR_ID);
            ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
            CANCODER = new WPI_CANCoder(CANCODER_ID);

            DRIVE_MOTOR.configFactoryDefault();
            ANGLE_MOTOR.restoreFactoryDefaults();
            CANCODER.configFactoryDefault();

            CANCODER.setPositionToAbsolute();
            CANCODER.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            CANCODER.configMagnetOffset(-ANGLE_OFFSET);

            ANGLE_MOTOR.setIdleMode(CANSparkMax.IdleMode.kBrake);

            DRIVE_MOTOR.config_kP(0, Constants.CHASSIS_DRIVE_MOTOR_KP);
            DRIVE_MOTOR.config_kD(0, Constants.CHASSIS_DRIVE_MOTOR_KD);

            double ANGLE_RATIO = 360 / Constants.CHASSIS_ANGLE_GEAR_RATIO;

            ANGLE_MOTOR.getPIDController().setP(Constants.CHASSIS_ANGLE_MOTOR_KP / ANGLE_RATIO);
            ANGLE_MOTOR.getPIDController().setD(Constants.CHASSIS_ANGLE_MOTOR_KD / ANGLE_RATIO);
            ANGLE_MOTOR.getPIDController().setFF(Constants.CHASSIS_ANGLE_MOTOR_KF / ANGLE_RATIO);
            ANGLE_MOTOR.getPIDController().setSmartMotionMaxVelocity(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY * ANGLE_RATIO, 0);
            ANGLE_MOTOR.getPIDController().setSmartMotionMaxAccel(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_ACCEL * ANGLE_RATIO, 0);
            ANGLE_MOTOR.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR * ANGLE_RATIO, 0);
            ANGLE_MOTOR.getEncoder().setPositionConversionFactor(ANGLE_RATIO);
            ANGLE_MOTOR.getEncoder().setVelocityConversionFactor(ANGLE_RATIO);

            syncPosition();
        }

        public void syncPosition() {
            ANGLE_MOTOR.getEncoder().setPosition(-CANCODER.getAbsolutePosition());
        }

        public final double getDriveGearRatio() {
            return switch (Constants.CHASSIS_DRIVE_GEAR_RATIO) {
                case L1 -> 8.14;
                case L2 -> 6.75;
                case L3 -> 6.12;
                case L4 -> 5.14;
            };
        }

        public void setDesiredDState (SwerveModuleState SWERVE_STATE) {
            SWERVE_STATE = optimize(SWERVE_STATE, getRotation());
            DRIVE_MOTOR.set(ControlMode.Velocity, SWERVE_STATE.speedMetersPerSecond * getDriveGearRatio()/10/Constants.CHASSIS_WHEEL_DIAMETER_METERS * 2048.0/Math.PI, DemandType.ArbitraryFeedForward, FEEDFORWARD.calculate(SWERVE_STATE.speedMetersPerSecond));
            ANGLE_MOTOR.getPIDController().setReference(-SWERVE_STATE.angle.getDegrees(), CANSparkMax.ControlType.kSmartMotion);
        }

        public static SwerveModuleState optimize (SwerveModuleState desiredState, Rotation2d current_angle) {
            double TARGET_ANGLE = placeInAppropriate0To360Scope(current_angle.getDegrees(), desiredState.angle.getDegrees());
            double TARGET_SPEED = desiredState.speedMetersPerSecond;
            double DELTA = TARGET_ANGLE - current_angle.getDegrees();
            if (Math.abs(DELTA) > 90) {
                TARGET_SPEED = -TARGET_SPEED;
                if (DELTA > 90) {
                    TARGET_ANGLE -= 180;
                } else {
                    TARGET_ANGLE += 180;
                }
            }
            return new SwerveModuleState(TARGET_SPEED, Rotation2d.fromDegrees(TARGET_ANGLE));
        }

        // TODO: I need an explanation on this function
        public static double placeInAppropriate0To360Scope(double currentAngle, double newAngle) {
            double lowerBound;
            double upperBound;
            double lowerOffset = currentAngle % 360;
            if (lowerOffset >= 0) {
                lowerBound = currentAngle - lowerOffset;
                upperBound = currentAngle + (360 - lowerOffset);
            } else {
                upperBound = currentAngle - lowerOffset;
                lowerBound = currentAngle - (360 + lowerOffset);
            }
            while (newAngle < lowerBound) {
                newAngle += 360;
            }
            while (newAngle > upperBound) {
                newAngle -= 360;
            }
            if (newAngle - currentAngle > 180) {
                newAngle -= 360;
            } else if (newAngle - currentAngle < -180) {
                newAngle += 360;
            }
            return newAngle;
        }

        public final SwerveModulePosition getPosition() {
            return new SwerveModulePosition(
                    DRIVE_MOTOR.getSelectedSensorPosition() / getDriveGearRatio() / 2048.0 * Math.PI * Constants.CHASSIS_WHEEL_DIAMETER_METERS,
                    getRotation());
        }

        private Rotation2d getRotation() {
            return Rotation2d.fromDegrees(-ANGLE_MOTOR.getEncoder().getPosition());
        }

        public void stop() {
            DRIVE_MOTOR.set(ControlMode.Velocity, 0);
            ANGLE_MOTOR.set(0);
        }
    }

    public enum GearRatio {
        L1,
        L2,
        L3,
        L4
    }
}
