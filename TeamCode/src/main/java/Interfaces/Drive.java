package Interfaces;
public interface Drive {
    public enum Direction {
        FORWARD,
        BACKWARD
    }
    public void Forward(double distanceInch, double power, double stopDistance);
    public void Backward(double distanceInch, double power);
    public void Straight(Direction direction, double distanceInch, double power, double stopDistance);
    public void Left(int degrees, double power);
    public void Right(int degrees, double power);
    public void BasicMotorControl(double right_stick_y);
    //public void StrafeRight(double distanceInch, double power);
    void StrafeRight(int SleepSeconds, double power);
    //public void StrafeLeft(double distanceInch, double power);

    void StrafeLeft(int SleepSeconds, double power);

    // public void Intake(Direction direction, double distanceInch, double power, double stopDistance);

    void Intake(Direction direction, int IntakeRuntime, double power);

    public void ShowTelemetry();
}
