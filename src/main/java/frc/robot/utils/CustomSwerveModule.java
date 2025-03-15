package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CustomSwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final PIDController steerPID;
    private final RelativeEncoder driveEncoder;
    private final CANcoder steerEncoder;

    public final int number;
    public final String name;

    public SwerveModuleState state;

    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig steerConfig;

    public CustomSwerveModule(int moduleNumber, String modulePosition) {
        number = moduleNumber;
        name = modulePosition;

        // * Initialize drive and steer motors based on module numbering
        driveMotor = new SparkMax(moduleNumber * 2, MotorType.kBrushless);
        steerMotor = new SparkMax(moduleNumber * 2 + 1, MotorType.kBrushless);

        // * Initialize the Motor Encoders for angle and positioning feedback
        steerEncoder = new CANcoder(moduleNumber + 10);
        driveEncoder = driveMotor.getEncoder();

        // * Initialize PID Controller for steering with gains
        steerPID = new PIDController(0.5, 0, 0.05);
        steerPID.enableContinuousInput(-0.5, 0.5);
        
        // * Impose Current Limit on Swerve Motors to prevent Brownout
        driveConfig = new SparkMaxConfig();
        steerConfig = new SparkMaxConfig();
        driveConfig.smartCurrentLimit(40);
        steerConfig.smartCurrentLimit(20);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateState(SwerveModuleState newState) {
        newState.optimize(Rotation2d.fromRotations(getSteerRotation()));

        // * Automatically applies new state
        setTargetAngle(newState.angle.getRotations());
        setDriveSpeed(newState.speedMetersPerSecond);
    }

    public void setTargetAngle(double targetAngle) {
        // * Calculate the error between the target and current angle
        double steerSpeed = steerPID.calculate(getSteerRotation(), targetAngle);

        // ? PID Diagnostics
        Logger.recordOutput("Current Angle", getSteerRotation());
        Logger.recordOutput("Target Angle", targetAngle);

        // * Set the steer motor speed based on error
        steerMotor.set(steerSpeed);
    }

    public void setDriveSpeed(double driveSpeed) {
        // * Set Module Drive Speed
        driveMotor.set(driveSpeed);
    }

    public void resetSteerEncoder() {
        // * Zero out Steer-Motor Encoder
        // * Used after adjusting wheel alignment manually
        steerEncoder.setPosition(0.0);
    }

    public void resetDriveEncoder() {
        // * Zero out Drive-Motor Encoder
        // * Used on Auto Init
        driveEncoder.setPosition(0.0);
    }

    public double getEncoderDistance() {
        // * Get Distance Traveled for Auto
        return driveEncoder.getPosition();
    }

    private double getSteerRotation() {
        // * Returns the Steer-Motor Encoder's angle in rotations
        return steerEncoder.getAbsolutePosition().refresh().getValueAsDouble();
    }
}