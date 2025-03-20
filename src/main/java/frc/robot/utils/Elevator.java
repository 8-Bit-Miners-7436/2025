package frc.robot.utils;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {
  // Initialize Motors, Encoder, and Limit Switches
  private final SparkMax coralMotor = new SparkMax(16, MotorType.kBrushless);
  private final SparkMax elevatorMotor = new SparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final DigitalInput upperLimitSwitch = new DigitalInput(9);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(0);

  public void moveToLevel(String level) {
    switch (level) {
      case "BASE"   -> setElevatorSpeed(-1);   //  A: Move to Bottom
      case "INTAKE" -> moveTo(10);             //  B: Move to Coral Intake
      case "L2"     -> moveTo(50);             //  X: Move to L2
      case "L3"     -> setElevatorSpeed(1);    //  Y: Move to L3 (Top)
      case "DOWN"   -> setElevatorSpeed(-0.5); // LB: Manually Move Down
      case "UP"     -> setElevatorSpeed(0.5);  // RB: Manually Move Up
      case "STOP"   -> setElevatorSpeed(0);
    }
  }

  // Moves Elevator Up or Down Till Within 2 Rotations of The Target
  private void moveTo(double targetRot) {
    double currentRot = elevatorEncoder.getPosition();
    if (currentRot < targetRot - 2) setElevatorSpeed(1);
    if (currentRot > targetRot + 2) setElevatorSpeed(-1);
  }

  // Moves Elevator Only if The Respective Limit Switch is Not Pressed
  public void setElevatorSpeed(double speed) {
    boolean limitHit = (speed > 0 ? upperLimitSwitch.get() : lowerLimitSwitch.get());
    if (limitHit) elevatorMotor.stopMotor();
    else          elevatorMotor.set(speed);
  }

  // Spins Coral Launcher Only if Speed Above 0.1 to Prevent Missfire
  public void setCoralSpeed(double speed) {
    coralMotor.set(speed > 0.1 ? speed : 0);
  }

  // Resets Encoder to Ensure Correct Height Detection
  public void resetEncoder() {
    elevatorEncoder.setPosition(0);
  }
}
