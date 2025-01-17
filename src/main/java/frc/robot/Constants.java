package frc.robot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.XboxController;

public class Constants {
    //motor CAN ids
    public static final int FRONT_RIGHT_MOTOR = 3;
    public static final int FRONT_LEFT_MOTOR = 5;
    public static final int BACK_RIGHT_MOTOR = 2;
    public static final int BACK_LEFT_MOTOR = 4;
    public static final int SHOOTING_MOTOR = 6;
    
    //electrical stuff
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 30;

    //what the robot does when not given commands (coasts)
    public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kCoast;

    public static final int SPEED = XboxController.Axis.kLeftY.value;
    public static final int ROTATION_SPEED = XboxController.Axis.kRightX.value;
    public static final int DRIVER_PORT = 0;
}