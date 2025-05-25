using System;
using UnityEngine;

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

    private Vector3 velocity;
    private float angularVelocity; // Rotation speed (rad/s) - simplified for Magnus effect
    private Vector3 angularVelocityVector; // More accurate representation of rotation
    private float currentDrag;
    private float currentSpin; // Used to influence angular velocity

    public event Action OnBounce;
    public event Action OnStop;

    private bool isMoving = false;
    private bool isGrounded = false;
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

    public void Launch(Vector3 direction, ShotData shot)
    {
        Vector3 flatDirection = direction.normalized;

        velocity = flatDirection * shot.power;
        velocity.y += shot.lift;

        currentSpin = shot.spin; // Use spin from ShotData to influence rotation
        currentDrag = shot.drag; // You might want to combine this with calculated air drag
        angularVelocity = currentSpin * 100f; // Example: Convert spin to angular velocity
        angularVelocityVector = transform.right * currentSpin * 100f; // Example: Spin around the right axis for sidespin

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
}

public enum ShotType
{
    Serve,
    Flat,
    Topspin,
    Slice,
    Lob,
    Volley
}

[Serializable]
public class ShotData
{
    public float power;
    public float lift;
    public float spin; // Lateral curve force
    public float drag; // Air resistance

/*    public ShotData(float power, float lift, float spin, float drag)
    {
        this.power = power;
        this.lift = lift;
        this.spin = spin;
        this.drag = drag;
    }*/
}
