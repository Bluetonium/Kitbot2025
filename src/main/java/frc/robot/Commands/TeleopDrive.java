package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import frc.robot.subSystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDrive extends Command{

    private DoubleSupplier speed;
    private DoubleSupplier rotationSpeed;
    private Chassis chassis;

    public TeleopDrive(Chassis chassis, DoubleSupplier speed, DoubleSupplier rotationSpeed)
    {
        this.chassis = chassis;
        addRequirements(chassis);
        this.speed = speed;
        this.rotationSpeed = speed;
    }



    @Override
    public void execute() {
        chassis.drive(speed.getAsDouble(), rotationSpeed.getAsDouble());

    }
}
