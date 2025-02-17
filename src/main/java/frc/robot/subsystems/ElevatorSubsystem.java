package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.XboxController;

public class ElevatorSubsystem extends SubsystemBase {

    XboxController m_functionsController = new XboxController(1);
    SparkMax Leader = new SparkMax(8, MotorType.kBrushless);
    SparkMax Follower = new SparkMax(9, MotorType.kBrushless);

    public ElevatorSubsystem() {
        /*
        * Set parameters that will apply to all SPARKs. We will also use this as
        * the left leader config.
        */
        // ...existing code...
    }

    // Method to set the speed of the motors
    public void setMotorSpeed(double speed) {
        Leader.set(speed);
        Follower.set(speed);
    }

    // Method to stop the motors
    public void stopMotors() {
        Leader.set(0);
        Follower.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add code here to update motor speeds based on controller input
        double speed = m_functionsController.getLeftY();
        setMotorSpeed(speed);
    }
}