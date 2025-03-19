package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final Translation2d flModulePosition = new Translation2d();
    private final Translation2d frModulePosition = new Translation2d();
    private final Translation2d blModulePosition = new Translation2d();
    private final Translation2d brModulePosition = new Translation2d();

    private final Pigeon2 gyro = new Pigeon2(9);

    private final SwerveModule frModule = new SwerveModule("FR Module",
    1, 5,
    false, true,
    5, 0);

    private final SwerveModule flModule = new SwerveModule("FL Module",
        2, 6,
        false, true,
        6, 0);

    private final SwerveModule blModule = new SwerveModule("BL Module",
        3, 7,
        false, true,
        7, 0);

    private final SwerveModule brModule = new SwerveModule("BR Module",
        4, 8,
        false, true,
        8, 0);


    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        flModulePosition, frModulePosition, blModulePosition, brModulePosition
    );

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics,
        getHeading(), 
        getModulePositions()
    );


    private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
    private final StructPublisher<Pose2d> position2dPublisher;

    public Swerve() {
        moduleStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("SmartDashboard/Swerve/SetModuleStates", SwerveModuleState.struct).publish();
        position2dPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Swerve/Pose2d", Pose2d.struct).publish();

    }

    public void drive(double vx, double vy, double omega) {
        // Chassis speeds packages the velocity x and y meters per second and radians per a second into one object
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
        // Uses inverse kinmatics to determine the states from the chassis speeds
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MaxAttainableSpeed);
        setModuleStates(states);
    }

    private double deadband(double input) {
        return (input < Constants.OperatorDeadband) ? 0.0 : input;
    }
    public void teleopDrive(DoubleSupplier percentX, DoubleSupplier percentY, DoubleSupplier percentRotation) {
        drive(deadband(percentX.getAsDouble()) * Constants.MaxAttainableSpeed,
              deadband(percentY.getAsDouble()) * Constants.MaxAttainableSpeed, 
              deadband(percentRotation.getAsDouble() * Constants.MaxAttainableSpeed));
    }

    public void setModuleStates(SwerveModuleState[] states) {
        flModule.setState(states[0]);
        frModule.setState(states[1]);
        blModule.setState(states[2]);
        brModule.setState(states[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            flModule.getState(),
            frModule.getState(), 
            blModule.getState(), 
            brModule.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            flModule.getPosition(),
            frModule.getPosition(),
            blModule.getPosition(),
            brModule.getPosition()
        };
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    /**
     * Angle read by the gyro
     * @return angle of the robot
     */
    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public void zeroGyro() {
        
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), getModulePositions());

        moduleStatePublisher.set(getModuleStates());
        position2dPublisher.set(getPose2d());
        SmartDashboard.putNumber("Swerve/vx meters per second", getSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/vy meters per second", getSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/omega radians per second", getSpeeds().omegaRadiansPerSecond);
    }
}