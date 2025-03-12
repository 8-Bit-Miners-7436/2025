package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CustomSwerveModule {
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final PIDController steerPID;
    private final CANcoder steerEncoder;

    public final int number;
    public final String name;

    private SwerveModuleState state;

    public CustomSwerveModule(int moduleNumber, String modulePosition) {
        number = moduleNumber;
        name = modulePosition;

        // * Initialize drive and steer motors based on module numbering
        driveMotor = new SparkMax(moduleNumber * 2, MotorType.kBrushless);
        steerMotor = new SparkMax(moduleNumber * 2 + 1, MotorType.kBrushless);

        // * Initialize the Steer Motor Encoder for steering angle feedback
        steerEncoder = new CANcoder(moduleNumber + 10);

        // * Initialize PID Controller for steering with gains
        steerPID = new PIDController(0.5, 0, 0.05);

        // * Initialize the steer motor encoder position using
        // * the current absolute position scaled by the gear ratio
        steerMotor.getEncoder().setPosition(getSteerRotation() / 360.0 * STEER_GEAR_RATIO);
    }

    public SwerveModuleState getState() {
        // * Allows other files to read module state
        return state;
    }

    public void updateState(SwerveModuleState newState) {
        this.state = newState;

        // * Automatically applies new state
        setTargetAngle(newState.angle.getRotations());
        setDriveSpeed(newState.speedMetersPerSecond);
    }

    public void setTargetAngle(double targetAngleDegrees) {
        // * Calculate the error between the target and current angle
        double error = targetAngleDegrees - getSteerRotation();

        // * Optimize error for wrap-around
        if (error > 0.5) {
            error -= 1;
        } else if (error < -0.5) {
            error += 1;
        }

        // * Set the steer motor speed based on error
        steerMotor.set(error);
    }

    public void setDriveSpeed(double driveSpeed) {
        driveMotor.set(driveSpeed);
    }

    public void resetEncoder() {
        // * Zeros out Steer-Motor Encoder
        // * Used after adjusting wheel alignment manually
        steerEncoder.setPosition(0.0);
    }

    private double getSteerRotation() {
        // * Returns the Steer-Motor Encoder's angle in rotations
        return steerEncoder.getAbsolutePosition().getValueAsDouble();
    }
}