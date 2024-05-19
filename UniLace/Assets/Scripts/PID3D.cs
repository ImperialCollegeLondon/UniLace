using UnityEngine;

public class PID3D
{
    public float kp;
    public float ki;
    public float kd;

    private Vector3 integral;
    private Vector3 previousError;
    public Vector3 target;
    public bool targetReached = false;
    public float tolerance;

    public PID3D(float kp, float ki, float kd, float tolerance = 0.01f)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.tolerance = tolerance;

        integral = Vector3.zero;
        previousError = Vector3.zero;
    }

    public void SetTarget(Vector3 target)
    {
        this.target = target;
    }

    public Vector3 Update(Vector3 current, float deltaTime)
    {
        Vector3 error = target - current;

        integral += error * deltaTime;
        Vector3 derivative = (error - previousError) / deltaTime;

        Vector3 output = kp * error + ki * integral + kd * derivative;

        previousError = error;

        if (error.magnitude < tolerance)
        {
            targetReached = true;
        }

        return output;
    }

    public void Reset()
    {
        integral = Vector3.zero;
        previousError = Vector3.zero;
        targetReached = false;
    }
}
