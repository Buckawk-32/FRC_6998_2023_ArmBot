// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveSubsystem;

public final class Constants
{
    public static final double CHASSIS_LENGTH_METERS = 0.62865;
    public static final double CHASSIS_WIDTH_METERS = 0.62865;
    public static final double CHASSIS_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

    public static final double CHASSIS_ANGLE_GEAR_RATIO = 21.42;

    public static final double CHASSIS_MAX_SPEED_METERS_PER_SEC = 8.0;
    public static final double CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 6.0;
    public static final double CHASSIS_MAX_ACCEL_METERS_PER_SEC = 5.0;
    public static final boolean CHASSIS_ENABLE_FIELD_ORIENTABLE_CONTROL = true;
    public static final double CHASSIS_DEADBAND_ZONE = 0.1;
    public static final SwerveSubsystem.GearRatio CHASSIS_DRIVE_GEAR_RATIO = SwerveSubsystem.GearRatio.L1;

    public static final Pose2d CHASSIS_INITIAL_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); // TODO: Remember to use actual start pos

    public static final double CHASSIS_DRIVE_MOTOR_KS = 0.077618 / 12;

    public static final double CHASSIS_DRIVE_MOTOR_KV = 0.89851 / 12;
    public static final double CHASSIS_DRIVE_MOTOR_KA = 0.18239 / 12;

    public static final double CHASSIS_DRIVE_MOTOR_KP = 0.05;
    public static final double CHASSIS_DRIVE_MOTOR_KD = 0.0;

    public static final double CHASSIS_ANGLE_MOTOR_KP = 0.0001;
    public static final double CHASSIS_ANGLE_MOTOR_KD = 0.0;
    public static final double CHASSIS_ANGLE_MOTOR_KF = 0.00017855;

    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY = 5760;
    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_ACCEL = 10000;
    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR = 0.01;

    // ---------------------------------------------------------------------

    public static final double AUTO_BALANCE_KP = 0.08;
    public static final double AUTO_BALANCE_KI = 0;
    public static final double AUTO_BALANCE_KD = 0;
    public static final double AUTO_BALANCE_START_SPEED_METERS_PER_SEC = -2;
    public static final double AUTO_BALANCE_TOLERANCE = 2;
    public static final double AUTO_BALANCE_WAIT_TIME = 2;

    // ---------------------------------------------------------------------

    public static final double CHASSIS_FRONT_LEFT_ANGLE_OFFSET = 0.0;
    public static final double CHASSIS_BACK_LEFT_ANGLE_OFFSET = 0.0;
    public static final double CHASSIS_BACK_RIGHT_ANGLE_OFFSET = 0.0;
    public static final double CHASSIS_FRONT_RIGHT_ANGLE_OFFSET = 0.0;

    public static final int CHASSIS_FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int CHASSIS_FRONT_LEFT_ANGLE_MOTOR_ID = 1;
    public static final int CHASSIS_FRONT_LEFT_CANCODER_ID = 1;

    public static final int CHASSIS_BACK_LEFT_DRIVE_MOTOR_ID = 2;
    public static final int CHASSIS_BACK_LEFT_ANGLE_MOTOR_ID = 2;
    public static final int CHASSIS_BACK_LEFT_CANCODER_ID = 2;

    public static final int CHASSIS_BACK_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int CHASSIS_BACK_RIGHT_ANGLE_MOTOR_ID = 3;
    public static final int CHASSIS_BACK_RIGHT_CANCODER_ID = 3;

    public static final int CHASSIS_FRONT_RIGHT_DRIVE_MOTOR_ID = 4;
    public static final int CHASSIS_FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
    public static final int CHASSIS_FRONT_RIGHT_CANCODER_ID = 4;

    // ---------------------------------------------------------------------

    public static final int ARM_TOP_MOTOR_ID = 6;
    public static final double ARM_TOP_ENCODER_OFFSET = 105;
    public static final double ARM_TOP_KF = 0.0028;
    public static final double ARM_TOP_KP = 0.007;
    public static final double ARM_TOP_MAX_ALLOWABLE_ERROR_DEG = 2;
    public static final double ARM_TOP_MAX_VELOCITY_DEG_PER_SEC = 90;
    public static final double ARM_TOP_MAX_ACCELERATION_DEG_PER_SQ = 90;

    // ---------------------------------------------------------------------

    public static final int ARM_BOTTOM_MOTOR_ID = 5;
    public static final double ARM_BOTTOM_ENCODER_OFFSET = 319.3 + 27;
    public static final double ARM_BOTTOM_KF = 0.0029077;
    public static final double ARM_BOTTOM_KP = 0.0034906;
    public static final double ARM_BOTTOM_MAX_ALLOWABLE_ERROR_DEG = 2;
    public static final double ARM_BOTTOM_MAX_VELOCITY_DEG_PER_SEC = 90;
    public static final double ARM_BOTTOM_MAX_ACCELERATION_DEG_PER_SQ = 90;

    // ---------------------------------------------------------------------

    public static final double ARM_TOP_GEARING = 80;
    public static final double ARM_BOTTOM_GEARING = 100 * 2;

    // ---------------------------------------------------------------------

    public static final int ARM_INTAKE_ROTATOR_MOTOR_ID = 7;
    public static final int ARM_INTAKE_ROTATOR_MOTOR_CURRENTLIMIT = 0;
    public static final double INTAKE_ROTATION_MOTOR_VOLTAGE_COMPENSATION = 0.0;
    public static final CANSparkMax.IdleMode INTAKE_ROTATION_MOTOR_IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double INTAKE_ROTATOR_MOTOR_KP = 0.0;
    public static final double INTAKE_ROTATOR_MOTOR_KI = 0.0;
    public static final double INTAKE_ROTATOR_MOTOR_KD = 0.0;
    public static final double INTAKE_ROTATOR_MOTOR_KF = 0.0;
    public static final double INTAKE_ROTATOR_DEADBAND = 0.05;

    // ---------------------------------------------------------------------

    public static final int ARM_INTAKE_SPIN_MOTOR_ID = 8;
    public static final int ARM_INTAKE_SPIN_MOTOR_CURRENTLIMIT = 0;
    public static final double INTAKE_SPIN_MOTOR_VOLTAGE_COMPENSATION = 0.0;
    public static final CANSparkMax.IdleMode INTAKE_SPIN_MOTOR_IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double INTAKE_SPIN_MOTOR_KP = 0.0;
    public static final double INTAKE_SPIN_MOTOR_KI = 0.0;
    public static final double INTAKE_SPIN_MOTOR_KD = 0.0;
    public static final double INTAKE_SPIN_MOTOR_KF = 0.0;

}
