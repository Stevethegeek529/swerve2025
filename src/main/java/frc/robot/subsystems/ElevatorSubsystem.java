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
    * Create new SPARK MAX configuration objects. These will store the
    * configuration parameters for the SPARK MAXes that we will set below.
    */
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig LeaderConfig = new SparkMaxConfig();
    SparkMaxConfig FollowerConfig = new SparkMaxConfig();

    /*
    * Set parameters that will apply to all SPARKs. We will also use this as
    * the left leader config.
    */
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    // Apply the global config and invert since it is on the opposite side
    LeaderConfig
        .apply(globalConfig);
        //.inverted(true);

    // Apply the global config and set the leader SPARK for follower mode
    FollowerConfig
        .apply(globalConfig)
        .follow(Leader);

    /*
    * Apply the configuration to the SPARKs.
    *
    * kResetSafeParameters is used to get the SPARK MAX to a known state. This
    * is useful in case the SPARK MAX is replaced.
    *
    * kPersistParameters is used to ensure the configuration is not lost when
    * the SPARK MAX loses power. This is useful for power cycles that may occur
    * mid-operation.
    */
    Leader.configure(LeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Follower.configure(FollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void RunElevator() {
    Leader.set(m_functionsController.getLeftY());
}


}
