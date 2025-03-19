package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private final String moduleName;
  private final TalonFX driveMotor;
  private final TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
  private final double driveGearRatio = 6.12;
  private final SparkMax steerMotor;
  private final SparkMaxConfig steerMotorConfig = new SparkMaxConfig();

  private final CANcoder encoder;
  private double encoderOffset;

  private Rotation2d desiredAngle = new Rotation2d();

  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);
  private double desiredVelocity = 0.0; // m/s

  public SwerveModule(
      String moduleName,
      int driveMotorID,
      int steerMotorID,
      boolean driveInverted,
      boolean steerInverted,
      int encoderID,
      double encoderOffset) {
    this.driveMotor = new TalonFX(driveMotorID);
    this.steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
    this.encoder = new CANcoder(steerMotorID);
    this.encoderOffset = encoderOffset;
    this.moduleName = moduleName;

    driveMotorConfiguration.Slot0 = new Slot0Configs().withKP(12.0).withKI(0.0).withKD(0.01);
    driveMotorConfiguration.MotorOutput.Inverted =
        (driveInverted)
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveMotorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    driveMotorConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
    driveMotorConfiguration.Feedback.SensorToMechanismRatio = driveGearRatio;
    driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveMotorConfiguration);

    steerMotorConfig.absoluteEncoder.inverted(true);

    steerMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(0.015, 0.0, 0.001, 0.0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true);

    steerMotorConfig
        .inverted(steerInverted)
        .closedLoopRampRate(0.25)
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake);
    steerMotor.configure(
        steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.MagnetOffset = encoderOffset;
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoder.getConfigurator().apply(canCoderConfiguration);
  }

  public void setState(SwerveModuleState state) {
    state.optimize(getAngle());
    desiredAngle = state.angle;
    desiredVelocity = state.speedMetersPerSecond;

    driveMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desiredVelocity));
    steerMotor
        .getClosedLoopController()
        .setReference(desiredAngle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(
        encoder.getAbsolutePosition().getValueAsDouble() + encoderOffset);
  }

  public double getVelocity() {
    return driveMotor.getVelocity().getValueAsDouble();
  }

  public SwerveModulePosition getPosition() {
    // rotations
    return new SwerveModulePosition(
        // the relative encoder on the talonfx drive motor divided by the number of rotations needed
        // to spin the drive wheel once
        // is the number of rotatiosn of the wheel, PI * wheel diamter = circumference
        // rotations of wheel * circumference of wheel = distance driven by the wheel
        (driveMotor.getPosition().getValueAsDouble() / Constants.DriveGearRatio)
            * Math.PI
            * Constants.WheelDiameter,
        getAngle());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Swerve/" + moduleName + "/current angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/desired angle", desiredAngle.getDegrees());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/current velocity", getVelocity());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/desired velocity", desiredVelocity);
  }
}
