package frc.robot.utils;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class Elevator {
    private final SparkMax elevatorMotor;
    private final RelativeEncoder elevatorMotorEncoder;
    private final SparkMax coralMotor;
    public final DigitalInput upperLimitSwitch;
    public final DigitalInput lowerLimitSwitch;

    public Elevator() {
        elevatorMotor = new SparkMax(15, MotorType.kBrushless);
        elevatorMotorEncoder = elevatorMotor.getEncoder();
        coralMotor = new SparkMax(16, MotorType.kBrushless);
        upperLimitSwitch = new DigitalInput(9);
        lowerLimitSwitch = new DigitalInput(0);
    }

    public int moveTo(int level) {
        double pos = getPosition();
        switch (level) {
            case 0:
                stopElevator();
                level = 0;
                break;
            case 1:
                lowerElevator();
                level = 1;
                break;
            case 2:
                if (pos < 9) {
                    raiseElevator();
                } else if (pos > 13) {
                    lowerElevator();
                } else {
                    stopElevator();
                }
                level = 2;
                break;
            case 3:
                if (pos < 48) {
                    raiseElevator();
                } else if (pos > 52) {
                    lowerElevator();
                } else {
                    stopElevator();
                }
                level = 3;
                break;
            case 4:
                raiseElevator();
                level = 4;
                break;
            case 5:
                lowerElevator();
                level = 0;
                break;
            case 6:
                raiseElevator();
                level = 0;
                break;
        }
        return level;
    }

    public void stopElevator() {
        elevatorMotor.stopMotor();
    }

    public double getPosition() {
        return elevatorMotorEncoder.getPosition();
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

    public void setCoralSpeed(double speed) {
        if (Math.abs(speed) > 0.01) {
            coralMotor.set(speed);
        } else {
            coralMotor.stopMotor();
        }
    }

    public void resetEncoder() {
        elevatorMotorEncoder.setPosition(0);
    }
}
