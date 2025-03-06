package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomSwerveModule {
    // ? May need to adjust gear ratio. (150/7) -> (8.14, 6.75, or 6.12)
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final PIDController steerPID;
    private final CANcoder steerEncoder;
    private final Integer moduleNumber;

    public CustomSwerveModule(int swerveModuleNumber) {
        moduleNumber = swerveModuleNumber;
        driveMotor = new SparkMax(moduleNumber * 2, MotorType.kBrushless);
        steerMotor = new SparkMax(moduleNumber * 2 + 1, MotorType.kBrushless);
        steerEncoder = new CANcoder(moduleNumber + 10);
        steerPID = new PIDController(0.5, 0, 0.05);
        steerPID.setPID(0.5, 0, 0.05);
        steerMotor.getEncoder().setPosition(getSteerRotation() / 360.0 * STEER_GEAR_RATIO);
    }

    public void setTargetAngle(double targetAngleDegrees) {
        double MotorSpeed = targetAngleDegrees - getSteerRotation();
        if (MotorSpeed > 0.5) {
            MotorSpeed -= 1;
        } else if (MotorSpeed < -0.5) {
            MotorSpeed += 1;
        }
        SmartDashboard.putNumber("Steer Motor Speed " + moduleNumber, MotorSpeed);
        steerMotor.set(MotorSpeed);
    }

    public void setDriveSpeed(double driveSpeed) {
        var maxSpeed = 2; // * (Meters per Second)
        driveMotor.set(driveSpeed * maxSpeed);
    }

    public void resetEncoder() {
        steerEncoder.setPosition(0.0);
    }

    public void stopMotor() {
        driveMotor.stopMotor();
    }

    // ** Internal Utilities **//
    private double getSteerRotation() {
        return steerEncoder.getAbsolutePosition().getValueAsDouble();
    }
}