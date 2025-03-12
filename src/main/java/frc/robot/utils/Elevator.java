package frc.robot.utils;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;

public class Elevator {
    private final SparkMax elevatorMotor;
    private final SparkMax coralMotor;
    private final DigitalInput upperLimitSwitch;
    private final DigitalInput lowerLimitSwitch;

    public Elevator() {
        elevatorMotor = new SparkMax(15, MotorType.kBrushless);
        coralMotor = new SparkMax(16, MotorType.kBrushless);
        upperLimitSwitch = new DigitalInput(9);
        lowerLimitSwitch = new DigitalInput(0);
    }

    public void stopElevator() {
        elevatorMotor.stopMotor();
    }
    
    public void raiseElevator() {
        if (upperLimitSwitch.get()) {
            elevatorMotor.stopMotor();
        } else {
            elevatorMotor.set(0.5);
        }
    }

    public void lowerElevator() {
        if (lowerLimitSwitch.get()) {
            elevatorMotor.stopMotor();
        } else {
            elevatorMotor.set(-0.5);
        }
    }

    public void setCoralSpeed(Double speed) {
        if (Math.abs(speed) > 0.01) {
            coralMotor.set(speed);
        } else {
            coralMotor.stopMotor();
        }
    }
}
