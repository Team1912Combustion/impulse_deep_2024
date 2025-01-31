package Interfaces;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class EHimu implements IGyro {
    public IMU Gyro;
    private Orientation _lastAngles = new Orientation();
    private double _globalAngle;

    private double lastHeading;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    public EHimu(HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        Initialize();
    }

    private void Initialize() {

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize Gyro
        Gyro = hardwareMap.get(IMU.class, "imu");

        Gyro.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("GC", "Gyro Calibrating. Do Not Move!");
        telemetry.update();
        //Gyro.calibrate();

        // Wait until the gyro calibration is complete
        timer.reset();
//        while (Gyro.isCalibrating()) {
//            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
//            telemetry.update();
//            sleep(50);
//        }

    }

    /**
     * Get current cumulative angle rotation from last reset.
     * https://stemrobotics.cs.pdx.edu/node/7268
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    @Override
    public double GetAngle() {
        /* We experimentally determined the Z axis is the axis we want to use for heading angle.
         * We have to process the angle because the imu works in euler angles so the Z axis is
         * returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
         * 180 degrees. We detect this transition and track the total cumulative angle of rotation. */


        /* Read dimensionalized data from the gyro. This gyro can report angular velocities
         * about all three axes. Additionally, it internally integrates the Z axis to
         * be able to report an absolute angular Z orientation. */
       // Orientation angles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        //angles   = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //double deltaAngle = angles.firstAngle - _lastAngles.firstAngle;


        double currentHeading = getHeading();
        double deltaAngle = currentHeading - lastHeading;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        _globalAngle += deltaAngle;

        lastHeading = currentHeading;

        return _globalAngle;
    }
    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = Gyro.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void ResetAngle() {
        //_lastAngles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        lastHeading = getHeading();
//        _globalAngle = 0;
        Gyro.resetYaw();
    }

    /**
     * Get current cumulative angle heading.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    @Override
    public int GetHeadingEH(){
        return (int)getHeading();
    }

    @Override
    public void ResetHeadingEH() {
        ResetAngle();
    }
}
