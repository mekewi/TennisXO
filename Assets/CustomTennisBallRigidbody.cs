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
    public float ballMass = 0.058f; // Standard tennis ball mass (kg)
    public float gravity = 9.81f;
    public float bounceDamping = 0.6f;
    public float minBounceVelocity = 1f;
    public float groundFriction = 0.8f; // Multiplier for horizontal velocity on bounce
    [Range(0f, 1f)]
    public float dragCoefficient = 0.05f; // Higher values increase drag
    public float airDensity = 1.225f; // Standard air density (kg/m^3)
    public float ballRadius = 0.0335f; // Radius of a tennis ball (meters)

    private Vector3 velocity;
    private Vector3 angularVelocityVector; // More accurate representation of rotation

    public event Action OnBounce;
    public event Action OnStop;
     public SwipeInput swipeInput; // Consider removing direct reference for better coupling

    private bool isMoving = false;
    private bool isGrounded = false;
    private bool hasBounced = false; // Renamed to avoid confusion with public bool bounce;

    void FixedUpdate() // Changed to FixedUpdate for consistent physics
    {
        if (!isMoving) return;

        float deltaTime = Time.fixedDeltaTime; // Use fixedDeltaTime

        ApplyGravity(deltaTime);
        ApplyDrag(deltaTime);

        transform.position += velocity * deltaTime;

        // Apply rotation based on angularVelocityVector
        transform.Rotate(angularVelocityVector * Mathf.Rad2Deg * deltaTime, Space.World);

        CheckGroundCollision();
        CheckStop();
    }
    /// <summary>
    /// Applies an instantaneous force (impulse) to the ball, affecting its velocity.
    /// </summary>
    /// <param name="force">The force vector to apply.</param>
    public void AddForce(Vector3 force)
    {
        // Calculate acceleration from force and mass (F = ma => a = F/m)
        Vector3 acceleration = force / ballMass;
        velocity += acceleration * Time.fixedDeltaTime;

        // Ensure the ball starts moving if it wasn't already
        if (!isMoving)
        {
            isMoving = true;
            isGrounded = false; // It's no longer grounded if a force is applied
        }
    }
    public float GetHeightAtHorizontalPosition(Vector3 initialPosition, Vector3 initialVelocity, Vector3 targetXZPosition)
    {
        // Extract horizontal components for initial position and velocity
        Vector3 initialPosXZ = new Vector3(initialPosition.x, 0, initialPosition.z);
        Vector3 initialVelXZ = new Vector3(initialVelocity.x, 0, initialVelocity.z);

        // Calculate the horizontal displacement vector needed to reach the target XZ
        Vector3 horizontalDisplacement = targetXZPosition - initialPosXZ;

        // Get the magnitude (speed) of the horizontal initial velocity
        float initialHorizontalSpeed = initialVelXZ.magnitude;

        // --- Step 1: Calculate the time to reach the target horizontal position ---

        // If horizontal speed is zero or very close to zero, the ball isn't moving horizontally.
        // It can only reach the target if the target is also at the initial horizontal position.
        if (Mathf.Abs(initialHorizontalSpeed) < 0.0001f) // Use a small epsilon for float comparison
        {
            if (horizontalDisplacement.magnitude < 0.0001f)
            {
                // Target XZ is essentially the initial XZ position, so height is initial height.
                return initialPosition.y;
            }
            else
            {
                // Ball cannot reach the target XZ position horizontally if its horizontal speed is zero.
                Debug.LogWarning("GetHeightAtHorizontalPosition: Initial horizontal velocity is zero, and target XZ is not initial XZ. Target is horizontally unreachable.");
                return float.NaN; // Not a Number, indicates an impossible calculation
            }
        }

        // Calculate the time it takes to cover the horizontal displacement.
        // We calculate this by finding how much 'initialVelXZ' needs to be scaled
        // to match 'horizontalDisplacement'. This time must be positive.
        float timeToReachXZ;

        // Check if the horizontal displacement is in the same direction as the initial horizontal velocity.
        // If the dot product is negative or very small, the target is in the "wrong" direction,
        // or too far off the path.
        if (Vector3.Dot(initialVelXZ.normalized, horizontalDisplacement.normalized) < 0.999f && horizontalDisplacement.magnitude > 0.0001f)
        {
            // If the angle between initial horizontal velocity and the displacement to target XZ is significant,
            // the target XZ position is not directly along the horizontal trajectory path.
            // For an ideal parabolic path (no drag), horizontal motion is a straight line.
            Debug.LogWarning($"GetHeightAtHorizontalPosition: Target horizontal position ({targetXZPosition}) is not directly on the straight horizontal trajectory path defined by initial velocity ({initialVelocity}). Ball will not pass through this point horizontally.");
            return float.NaN;
        }

        // Time = (Magnitude of horizontal displacement) / (Magnitude of horizontal velocity)
        timeToReachXZ = horizontalDisplacement.magnitude / initialHorizontalSpeed;

        // If time is negative (e.g., target is behind ball's initial direction), it's physically impossible.
        if (timeToReachXZ < 0)
        {
            Debug.LogWarning("GetHeightAtHorizontalPosition: Calculated time to reach target XZ is negative. Target is behind the ball's horizontal direction.");
            return float.NaN;
        }

        // --- Step 2: Use the calculated time to find the vertical height ---

        // Get the initial vertical position and initial vertical velocity component
        float y0 = initialPosition.y;
        float v0y = initialVelocity.y;

        // Use the kinematic equation for vertical displacement (assuming constant gravity, no drag)
        // y = y0 + v0y*t + 0.5 * a_y * t^2
        // where a_y = -gravity (since gravity acts downwards)
        float height = y0 + (v0y * timeToReachXZ) + (0.5f * -gravity * timeToReachXZ * timeToReachXZ);

        return height;
    }
    public float GetHeightAtSpecificZPosition(Vector3 initialPosition, Vector3 initialVelocity, float targetZPosition)
    {
        // Calculate the Z-axis displacement needed to reach targetZPosition
        float zDisplacement = targetZPosition - initialPosition.z;

        // Get the Z-component of the initial velocity
        float v0z = initialVelocity.z;

        // --- Step 1: Calculate the time to reach the target Z position ---

        // If v0z is zero or very close to zero, the ball isn't moving along the Z-axis.
        // It can only reach the target Z if the target Z is also the initial Z position.
        if (Mathf.Abs(v0z) < 0.0001f) // Use a small epsilon for float comparison
        {
            if (Mathf.Abs(zDisplacement) < 0.0001f)
            {
                // Target Z is essentially the initial Z position, so height is initial height.
                return initialPosition.y;
            }
            else
            {
                // Ball cannot reach the target Z position if its Z velocity is zero,
                // and the target Z is not the starting Z.
                Debug.LogWarning("GetHeightAtSpecificZPosition: Initial Z velocity is zero, and target Z is not initial Z. Target is Z-unreachable.");
                return float.NaN; // Indicates an impossible calculation
            }
        }

        // Calculate time to reach the target Z position: Time = Z_displacement / V0z
        float timeToReachZ = zDisplacement / v0z;

        // If time is negative, it means the target Z position is behind the ball's Z direction.
        if (timeToReachZ < 0)
        {
            Debug.LogWarning("GetHeightAtSpecificZPosition: Calculated time to reach target Z is negative. Target is behind the ball's Z direction.");
            return float.NaN;
        }

        // --- Step 2: Use the calculated time to find the vertical height ---

        // Get the initial vertical position and initial vertical velocity component
        float y0 = initialPosition.y;
        float v0y = initialVelocity.y; // The Y component of the initial velocity vector

        // Apply the kinematic equation for vertical displacement:
        // y = initial_y + (initial_vertical_velocity * time) + (0.5 * acceleration_y * time^2)
        // Here, acceleration_y is -gravity (since gravity acts downwards and 'up' is positive).
        float height = y0 + (v0y * timeToReachZ) + (0.5f * -gravity * timeToReachZ * timeToReachZ);

        return height;
    }
    public float CorrectInitialVelocityY(Vector3 position, Vector3 velocity, float zTarget, float yMin, float gravity = 9.81f)
    {
        float dz = zTarget - position.z;
        float vz = velocity.z;

        if (Mathf.Approximately(vz, 0f))
        {
            Debug.LogWarning("Z velocity is zero — will never reach target Z.");
            return 0;
        }

        float t = dz / vz;

        float currentY = position.y + velocity.y * t - 0.5f * gravity * t * t;


        return currentY;
    }
    public float CalculateExtraTimeToReachYMin(
    float y0,
    float vy,
    float yTarget,
    float yMin,
    float gravity = 9.81f)
    {
        // Time to reach yTarget
        float a = -0.5f * gravity;
        float b = vy;
        float c = y0 - yTarget;
        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0f) return -1f; // Won’t reach yTarget

        float sqrtDisc = Mathf.Sqrt(discriminant);
        float t0 = Mathf.Max((-b + sqrtDisc) / (2 * a), (-b - sqrtDisc) / (2 * a));

        // Velocity when it reaches yTarget
        float v1 = vy - gravity * t0;

        // Now compute time to fall from yTarget to yMin
        float a2 = -0.5f * gravity;
        float b2 = v1;
        float c2 = yTarget - yMin;

        float disc2 = b2 * b2 - 4 * a2 * c2;
        if (disc2 < 0f) return -1f; // Won’t reach yMin from there

        float sqrtDisc2 = Mathf.Sqrt(disc2);
        float t1 = Mathf.Max((-b2 + sqrtDisc2) / (2 * a2), (-b2 - sqrtDisc2) / (2 * a2));

        return t1; // this is the additional time needed after reaching yTarget
    }

    public float IsBallWillHit(
        Vector3 startPos,
        Vector3 targetPos,
        float netHeight,
        float value // either flightTime or arcHeight depending on mode
    )
    {
        var calculatedVelocity = GetVelocity(startPos,targetPos,ShotControlMode.UseFlightTime,value);
        var height = GetHeightAtSpecificZPosition(startPos, calculatedVelocity, 0);
        Debug.Log("GetHeightAtSpecificZPosition > " + height);
        return height;
    }
    public void SetVelocity
    (
        Vector3 initialVelocity
    )
    {
        velocity = initialVelocity;

        isMoving = true;
        isGrounded = false;
        hasBounced = false; // Reset bounce flag
        float initialSpeedMPS = velocity.magnitude;
        Debug.Log($"Initial Shot Speed: {initialSpeedMPS:F2} m/s ({initialSpeedMPS * 3.6f:F2} km/h) + " + velocity);

    }
    public Vector3 GetVelocity
    (
        Vector3 startPos,
        Vector3 targetPos,
        ShotControlMode mode,
        float value // either flightTime or arcHeight depending on mode
    )
    {
        transform.position = startPos; // Ensure ball starts at the correct position
        Vector3 displacement = targetPos - startPos;
        Vector3 displacementXZ = new Vector3(displacement.x, 0f, displacement.z);
        float g = gravity;
        float flightTime;
        float vy;
        Vector3 velocityXZ;

        if (mode == ShotControlMode.UseFlightTime)
        {
            flightTime = value;

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
            flightTime = timeToPeak + timeDown;

            velocityXZ = new Vector3(
                displacementXZ.x / flightTime,
                0f,
                displacementXZ.z / flightTime
            );
        }

        return velocityXZ + (Vector3.up * vy);
    }
    public void ShootTo(
        Vector3 startPos,
        Vector3 targetPos,
        ShotControlMode mode,
        float value // either flightTime or arcHeight depending on mode
    )
    {
        transform.position = startPos; // Ensure ball starts at the correct position
        Vector3 displacement = targetPos - startPos;
        Vector3 displacementXZ = new Vector3(displacement.x, 0f, displacement.z);
        float g = gravity;
        float flightTime;
        float vy;
        Vector3 velocityXZ;

        if (mode == ShotControlMode.UseFlightTime)
        {
            flightTime = value;

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
            flightTime = timeToPeak + timeDown;

            velocityXZ = new Vector3(
                displacementXZ.x / flightTime,
                0f,
                displacementXZ.z / flightTime
            );
        }

        velocity = velocityXZ + (Vector3.up * vy);

        isMoving = true;
        isGrounded = false;
        hasBounced = false; // Reset bounce flag
        float initialSpeedMPS = velocity.magnitude;
        Debug.Log($"Initial Shot Speed: {initialSpeedMPS:F2} m/s ({initialSpeedMPS * 3.6f:F2} km/h) + "+ velocity);

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
            float area = Mathf.PI * ballRadius * ballRadius;
            Vector3 dragForce = 0.5f * dragCoefficient * airDensity * area * speed * speed * -velocity.normalized;
            velocity += (dragForce / ballMass) * deltaTime; // Apply force divided by mass
        }
    }
    private void CheckGroundCollision()
    {
        if (transform.position.y <= ballRadius && velocity.y < 0)
        {
            transform.position = new Vector3(transform.position.x, ballRadius, transform.position.z);
            velocity.y *= -bounceDamping;
            velocity.x *= groundFriction;
            velocity.z *= groundFriction;

            // Reduce angular velocity on bounce
            angularVelocityVector *= bounceDamping;

            if (Mathf.Abs(velocity.y) < minBounceVelocity)
            {
                velocity.y = 0f;
                isGrounded = true;
            }

            if (!hasBounced)
            {
                hasBounced = true;
                Debug.Log("Bounce > " + (Time.time - swipeInput.startTime)); // Decouple if swipeInput isn't always available
                OnBounce?.Invoke();
            }
        }
        else if (transform.position.y <= ballRadius && velocity.y >= 0)
        {
            transform.position = new Vector3(transform.position.x, ballRadius, transform.position.z);
            velocity.y = Mathf.Max(0f, velocity.y);
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
        angularVelocityVector = Vector3.zero;
        isMoving = false;
        isGrounded = false;
        hasBounced = false;
    }

    void OnDrawGizmos()
    {
        if (!Application.isPlaying || !isMoving) return;

        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + velocity.normalized * 2f); // Increased length for visibility
        Gizmos.DrawSphere(transform.position + velocity.normalized * 2f, 0.1f); // Arrowhead

        Gizmos.color = Color.blue;
        Vector3 magnusDir = Vector3.Cross(angularVelocityVector, velocity).normalized;
        Gizmos.DrawLine(transform.position, transform.position + magnusDir * 2f); // Increased length for visibility
        Gizmos.DrawSphere(transform.position + magnusDir * 2f, 0.1f); // Arrowhead

        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, transform.position + angularVelocityVector.normalized * 1f); // Spin axis visualization
        Gizmos.DrawSphere(transform.position + angularVelocityVector.normalized * 1f, 0.05f);
    }
}