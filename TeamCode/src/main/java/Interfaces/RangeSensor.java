package Interfaces;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSensor implements IDistance {

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    /**
     * Range sensor constructor
     * In Java, a constructor is a special method that is called when an object of a class is created. Its purpose is to initialize the object's state, typically by assigning initial values to instance variables or performing other setup tasks.
     * @param hardwareMap
     * @param telemetry
     */
    public RangeSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        Initialize();
    }

    /**
    * Initializes the range sensor
    */
    private void Initialize(){
        // get a reference to our compass
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }

    /**
     * Get the distance measured by the range sensor in centimeters.
     *
     * @return The distance in centimeters.
     */
    @Override
    public double GetDistance() {
        return rangeSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Shows telemetry for the range sensor
     *
     */
    @Override
    public void ShowTelemetry() {
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
