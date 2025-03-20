package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

    public CustomSwerveModule(int moduleNumber, String modulePosition) {
        // Initialize Drive and Steer Motors Based on Module Numbering
        driveMotor = new SparkMax(moduleNumber * 2, MotorType.kBrushless);
        steerMotor = new SparkMax(moduleNumber * 2 + 1, MotorType.kBrushless);

        // Initialize the Motor Encoders for Angle and Positioning Feedback
        steerEncoder = new CANcoder(moduleNumber + 10);
        driveEncoder = driveMotor.getEncoder();

        // Initialize PID Controller
        steerPID = new PIDController(0.5, 0, 0.05);
        steerPID.enableContinuousInput(-0.5, 0.5);

        // Impose Current Limit on Motors to Prevent Brownout
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(30);
        driveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateState(SwerveModuleState newState) {
        // Get Steer Motor Angle in Rotations
        double steerMotorRot = steerEncoder.getAbsolutePosition().refresh().getValueAsDouble();

        // Fix Wrap Around Issues
        newState.optimize(Rotation2d.fromRotations(steerMotorRot));

        // Calculate Speed Based on Error between the Target and Current Angle
        double steerSpeed = steerPID.calculate(steerMotorRot, newState.angle.getRotations());

        // Pass Finalized Values to Motors
        steerMotor.set(steerSpeed);
        driveMotor.set(newState.speedMetersPerSecond);
    }

    // Set Drive Motor Speed
    public void setDriveSpeed(double driveSpeed) {
        driveMotor.set(driveSpeed);
    }

    // Zero Out Motor Encoders to Correct Alignment
    public void resetEncoders() {
        steerEncoder.setPosition(0);
        driveEncoder.setPosition(0);
    }

    // Get Distance Traveled for Auto
    public double getEncoderDistance() {
        return driveEncoder.getPosition();
    }
}