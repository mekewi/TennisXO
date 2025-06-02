using System;
using UnityEngine;

public enum ShotControlMode
{
    UseFlightTime,
    UseArcHeight
}
public enum SpinType
{
    None,
    Topspin,
    Backspin,
    LeftSidespin,
    RightSidespin
}

public class CustomTennisBallRigidbody : MonoBehaviour
{
    [Header("Physics Settings")]
    public float gravity = 9.81f;
    public float bounceDamping = 0.6f;
    public float minBounceVelocity = 1f;
    public float groundFriction = 0.8f; // Multiplier for horizontal velocity on bounce
    [Range(0f, 1f)]
    public float dragCoefficient = 0.05f; // Higher values increase drag
    public float airDensity = 1.225f; // Standard air density (kg/m^3)
    public float ballRadius = 0.0335f; // Radius of a tennis ball (meters)
    public float spinMagnusCoefficient = 0.005f; // Controls the strength of the Magnus effect
    public float speed = 0.75f; // Controls the strength of the Magnus effect

    private Vector3 velocity;
    private float angularVelocity; // Rotation speed (rad/s) - simplified for Magnus effect
    private Vector3 angularVelocityVector; // More accurate representation of rotation
    private float currentSpin; // Used to influence angular velocity

    public event Action OnBounce;
    public event Action OnStop;
    public SwipeInput swipeInput;

    private bool isMoving = false;
    private bool isGrounded = false;
    private bool bounce = false;
    void Update()
    {
        if (!isMoving) return;

        float deltaTime = Time.deltaTime;

        ApplyGravity(deltaTime);
        ApplyDrag(deltaTime);
        ApplyMagnusForce(deltaTime);
        transform.position += velocity * deltaTime;
        // Apply rotation (simplified - could be more complex with angularVelocityVector)
        transform.Rotate(Vector3.forward, angularVelocity * Mathf.Rad2Deg * deltaTime);

        CheckGroundCollision();
        CheckStop();
    }

    public void SetSpin(SpinType spinType, float spinAmount = 1f)
    {
        currentSpin = spinAmount;

        switch (spinType)
        {
            case SpinType.Topspin:
                angularVelocityVector = transform.right * currentSpin * 100f;
                break;
            case SpinType.Backspin:
                angularVelocityVector = -transform.right * currentSpin * 100f;
                break;
            case SpinType.LeftSidespin:
                angularVelocityVector = -transform.up * currentSpin * 100f;
                break;
            case SpinType.RightSidespin:
                angularVelocityVector = transform.up * currentSpin * 100f;
                break;
            default:
                angularVelocityVector = Vector3.zero;
                break;
        }

        // Optional: Also store magnitude for simplified rotation
        angularVelocity = currentSpin * 100f;
    }



    public void ShootTo(
    Vector3 startPos,
    Vector3 targetPos,
    ShotControlMode mode,
    float value // either flightTime or arcHeight depending on mode
)
    {
        Vector3 displacement = targetPos - startPos;
        Vector3 displacementXZ = new Vector3(displacement.x, 0f, displacement.z);
        float g = gravity;
        float flightTime; // Declare flightTime here
        float vy;
        Vector3 velocityXZ;

        if (mode == ShotControlMode.UseFlightTime)
        {
            flightTime = value;

            // ✅ Preserve direction in X and Z
            velocityXZ = new Vector3(
                displacementXZ.x / flightTime,
                0f,
                displacementXZ.z / flightTime
            );

            vy = (targetPos.y - startPos.y + 0.5f * g * flightTime * flightTime) / flightTime;
        }
        else // UseArcHeight
        {
            float arcHeight = value;

            float peakY = Mathf.Max(startPos.y, targetPos.y) + arcHeight;
            float timeToPeak = Mathf.Sqrt(2f * (peakY - startPos.y) / g);
            vy = g * timeToPeak;

            float timeDown = Mathf.Sqrt(2f * (peakY - targetPos.y) / g);
            flightTime = timeToPeak + timeDown; // Assign flightTime here

            velocityXZ = new Vector3(
                displacementXZ.x / flightTime,
                0f,
                displacementXZ.z / flightTime
            );
        }

        velocity = velocityXZ + Vector3.up * vy;

        isMoving = true;
        isGrounded = false;

    }

    private void ApplyGravity(float deltaTime)
    {
        velocity.y -= gravity * deltaTime;
    }

    private void ApplyDrag(float deltaTime)
    {
        float speed = velocity.magnitude;
        if (speed > 0)
        {
            // More realistic drag force: F_drag = 0.5 * dragCoefficient * airDensity * area * speed^2 * -velocity.normalized
            float area = Mathf.PI * ballRadius * ballRadius;
            Vector3 dragForce = 0.5f * dragCoefficient * airDensity * area * speed * speed * -velocity.normalized;
            velocity += dragForce * deltaTime;
        }
    }

    private void ApplyMagnusForce(float deltaTime)
    {
        // Magnus effect: Force perpendicular to both velocity and spin axis
        // F_magnus = spinMagnusCoefficient * radius * airDensity * area * (angularVelocityVector x velocity)
        float area = Mathf.PI * ballRadius * ballRadius;
        Vector3 magnusForce = spinMagnusCoefficient * ballRadius * airDensity * area * Vector3.Cross(angularVelocityVector, velocity);
        velocity += magnusForce * deltaTime;
    }

    private void CheckGroundCollision()
    {
        if (transform.position.y <= ballRadius && velocity.y < 0)
        {
            // Approximate collision with a flat ground at y = 0 (considering ball radius)
            transform.position = new Vector3(transform.position.x, ballRadius, transform.position.z);
            velocity.y *= -bounceDamping;
            velocity.x *= groundFriction;
            velocity.z *= groundFriction;

            // Reduce angular velocity on bounce (simplified)
            angularVelocity *= bounceDamping;
            angularVelocityVector *= bounceDamping;

            if (Mathf.Abs(velocity.y) < minBounceVelocity)
            {
                velocity.y = 0f;
                isGrounded = true;
            }
            if (!bounce)
            {
                bounce = true;
                Debug.Log("Bounce > " + (Time.time - swipeInput.startTime));
            }
            OnBounce?.Invoke();
        }
        else if (transform.position.y <= ballRadius && velocity.y >= 0)
        {
            // Prevent sinking below the ground
            transform.position = new Vector3(transform.position.x, ballRadius, transform.position.z);
            velocity.y = Mathf.Max(0f, velocity.y); // Ensure no downward velocity when grounded
            isGrounded = true;
        }
        else
        {
            isGrounded = false;
        }
    }

    private void CheckStop()
    {
        if (isGrounded && velocity.magnitude < 0.1f)
        {
            isMoving = false;
            velocity = Vector3.zero;
            angularVelocity = 0f;
            angularVelocityVector = Vector3.zero;
            OnStop?.Invoke();
        }
    }

    public bool IsMoving() => isMoving;

    public Vector3 GetVelocity() => velocity;

    public void ResetBall(Vector3 position)
    {
        transform.position = position;
        velocity = Vector3.zero;
        angularVelocity = 0f;
        angularVelocityVector = Vector3.zero;
        isMoving = false;
        isGrounded = false;
    }
    void OnDrawGizmos()
    {
        if (!Application.isPlaying || !isMoving) return;

        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + velocity.normalized);

        Gizmos.color = Color.blue;
        Vector3 magnusDir = Vector3.Cross(angularVelocityVector, velocity).normalized;
        Gizmos.DrawLine(transform.position, transform.position + magnusDir);
    }
    
}