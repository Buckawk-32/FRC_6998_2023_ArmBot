package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    public final AHRS
}
