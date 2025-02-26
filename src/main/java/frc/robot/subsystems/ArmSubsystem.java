package frc.robot.subsystems;

import frc.robot.subsystems.ArmSubsystem;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final SparkMax intakeMotor;
    private final XboxController m_functionsController;
    private final AbsoluteEncoder m_armEncoder;

  public ArmSubsystem(XboxController controller) {
    // Constructor for ArmSubsystem
    armMotor = new SparkMax(6, MotorType.kBrushless);
    intakeMotor = new SparkMax(7, MotorType.kBrushless);
    m_armEncoder = armMotor.getAbsoluteEncoder();

    m_functionsController = controller;
  }

  // Method to set the speed of the motors
  public void setArmMotorSpeed(double speed) {
    double position = m_armEncoder.getPosition();

    //ensure the arm does not go below the lower limit
    if ((position <= 0.05 && speed < 0) || (position >= 0.3 && speed > 0)) {
      armMotor.set(0); //stop the motor if it tries to go past the limits
    } else {
      speed *= 0.5; //reduce the speed of the arm motor
      armMotor.set(speed); // set the motor speed if within limits
    }
  }
    //armMotor.set(speed);}
  // Method to set the speed of the intake motor
  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }

  // Method to stop the motors
  public void stopMotors() {
    // Method to stop the motors
    armMotor.set(0);
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      // Read the right joystick Y-axis value from the Xbox controller
      double armSpeed = m_functionsController.getRightY();
      System.out.println("Controller Right Joystick Y-axis value: " + armSpeed); // Debug statement

      // Display the controller value on the SmartDashboard
      SmartDashboard.putNumber("Controller Right Joystick Y-axis value", armSpeed);

      // Set the motor speed based on the joystick input
      setArmMotorSpeed(armSpeed);

      boolean leftBumper = m_functionsController.getLeftBumper();
      Boolean rightBumper = m_functionsController.getRightBumper();

      if (leftBumper) {
        setIntakeMotorSpeed(-0.5); // backward
      } else if (rightBumper) {
        setIntakeMotorSpeed(.5); // forward
      } else {
        setIntakeMotorSpeed(0); //stop intake motor
      }

      SmartDashboard.putNumber("intake motor speed", intakeMotor.get());
      SmartDashboard.putNumber("arm motor speed", armMotor.get());
      SmartDashboard.putNumber("arm motor position", m_armEncoder.getPosition());
      }
  }
