using System;
using System.Collections.Generic;
using UnityEngine;
public struct TrajectoryPoint
{
    public Vector3 position;
    public Vector3 velocity;
    public float time;
    public bool bounced;
    internal Vector3 angularVelocity;
}

[RequireComponent(typeof(Rigidbody))] // Ensures a Rigidbody is always present
[RequireComponent(typeof(SphereCollider))] // Ensures a SphereCollider is always present
public class TennisBallLauncher : MonoBehaviour, ICustomTennisBallRigidbody
{
    [Header("Physics Settings")]
    public float ballMass = 0.058f; // Standard tennis ball mass (kg)
    public float bounceDamping = 0.6f; // How much energy is lost on bounce (via Physic Material Bounciness)
    public float groundFriction = 0.8f; // How much horizontal velocity is retained on bounce (via Physic Material Dynamic Friction)
    [Range(0f, 1f)]
    public float dragCoefficient = 0.05f; // For air resistance (via Rigidbody.drag)
    public float angularDragCoefficient = 0.05f; // For angular air resistance (via Rigidbody.angularDrag)
    public float ballRadius = 0.0335f; // Radius of a tennis ball (meters) - used for SphereCollider

    [Header("Spin Settings")]
    public float spinForceMultiplier = 0.01f; // Adjust this to control the strength of Magnus effect

    // Private reference to Unity's Rigidbody
    public Rigidbody rb;

    // We'll still store initial velocity for trajectory calculations
    private Vector3 initialShotVelocity;

    public event Action OnBounce;
    public event Action OnStop;
    public SwipeInput swipeInput; // Consider removing direct reference for better coupling
    public PhysicsMaterial groundPhysicMaterial;
    public PhysicsMaterial ballPhysicMaterialInstance; // Reference to the actual PhysicMaterial instance on the ball's collider

    private bool isMoving = false;
    private bool hasBounced = false;
    bool debugHeight;
    float oldYPosition;
    public Action onReachHight;
    Vector3 start;
    public void Pause() 
    {
        rb.isKinematic = true;
    }
    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        // Configure Rigidbody for Unity Physics
        //rb.mass = ballMass;
        rb.useGravity = false; // Let Unity handle gravity
        rb.isKinematic = true; // Allow Unity physics to control movement
/*        rb.linearDamping = dragCoefficient; // Set linear drag
        rb.angularDamping = angularDragCoefficient; // Set angular drag
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

        // Configure Sphere Collider
        SphereCollider sphereCollider = GetComponent<SphereCollider>();
        if (sphereCollider != null)
        {
            sphereCollider.radius = ballRadius;
            // Create and assign a PhysicMaterial for bounce and friction
            PhysicsMaterial ballPhysicMaterial = new PhysicsMaterial
            {
                bounciness = bounceDamping,
                dynamicFriction = groundFriction, // Friction when moving
                staticFriction = groundFriction,   // Friction when stationary
                frictionCombine = PhysicsMaterialCombine.Multiply, // How frictions combine
                bounceCombine = PhysicsMaterialCombine.Multiply    // How bounciness combines
            };
            sphereCollider.material = ballPhysicMaterial;
        }
        else
        {
            Debug.LogError("SphereCollider missing on Tennis Ball!");
        }*/
    }

    void FixedUpdate()
    {
        // Unity's Rigidbody handles gravity, drag, and linear movement directly.
        // We only need to apply custom forces like Magnus effect here.

        // Check if ball is moving to apply spin and stop logic
        if (isMoving)
        {
            // Apply Magnus force if there's angular velocity (spin) and linear velocity
            ApplyMagnusForce();

            CheckStop();
            if (debugHeight)
            {
                if (transform.position.y < oldYPosition)
                {
                    Debug.Log("Height = " + transform.position.y);
                    Debug.Log("Height mag= " + (transform.position - start).magnitude);
                    debugHeight = false;
                    onReachHight?.Invoke();
                    Pause();
                    return;
                }
                oldYPosition = transform.position.y;
            }
        }
    }

    private void ApplyMagnusForce()
    {
        // Magnus force is proportional to the cross product of angular velocity and linear velocity
        // F_magnus = C * (omega x v)
        // C is a constant (spinForceMultiplier takes care of this)
        Vector3 magnusForce = Vector3.Cross(rb.angularVelocity, rb.linearVelocity) * spinForceMultiplier;
        rb.AddForce(magnusForce, ForceMode.Force); // Apply as a continuous force
    }


    /// <summary>
    /// Applies an instantaneous force (impulse) to the ball, affecting its velocity.
    /// In Unity physics, this is done via Rigidbody.AddForce with ForceMode.Impulse.
    /// </summary>
    /// <param name="force">The force vector to apply.</param>
    public void AddForce(Vector3 force)
    {
        rb.AddForce(force, ForceMode.Impulse); // Use ForceMode.Impulse for instantaneous force

        if (!isMoving)
        {
            isMoving = true;
            hasBounced = false; // Reset bounce flag
        }
    }

    // --- Trajectory Calculation Methods (remain mostly the same, as they are pure math) ---
    // These methods calculate the *initial* velocity needed to reach a target,
    // they don't directly move the ball.

    public float GetHeightAtHorizontalPosition(Vector3 initialPosition, Vector3 initialVelocity, Vector3 targetXZPosition)
    {
        // ... (This method remains the same, as it's a kinematic calculation) ...
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
        // where a_y = -Physics.gravity.y (since gravity acts downwards)
        float height = y0 + (v0y * timeToReachXZ) + (0.5f * Physics.gravity.y * timeToReachXZ * timeToReachXZ);

        return height;
    }

    public float GetHeightAtSpecificZPosition(Vector3 initialPosition, Vector3 initialVelocity, float targetZPosition)
    {
        // ... (This method remains the same, as it's a kinematic calculation) ...
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
        // Here, acceleration_y is Physics.gravity.y (since Unity's Physics.gravity is typically (0, -9.81, 0))
        float height = y0 + (v0y * timeToReachZ) + (0.5f * Physics.gravity.y * timeToReachZ * timeToReachZ);

        return height;
    }

    public float CorrectInitialVelocityY(Vector3 position, Vector3 velocity, float zTarget, float yMin) // Removed gravity param
    {
        float dz = zTarget - position.z;
        float vz = velocity.z;

        if (Mathf.Approximately(vz, 0f))
        {
            Debug.LogWarning("Z velocity is zero — will never reach target Z.");
            return 0;
        }

        float t = dz / vz;

        // Use Unity's gravity
        float currentY = position.y + velocity.y * t + 0.5f * Physics.gravity.y * t * t;


        return currentY;
    }

    public float CalculateExtraTimeToReachYMin(
        float y0,
        float vy,
        float yTarget,
        float yMin) // Removed gravity param
    {
        float g = -Physics.gravity.y; // Convert Unity's gravity to a positive scalar for this formula

        // Time to reach yTarget
        float a = -0.5f * g;
        float b = vy;
        float c = y0 - yTarget;
        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0f) return -1f; // Won’t reach yTarget

        float sqrtDisc = Mathf.Sqrt(discriminant);
        float t0 = Mathf.Max((-b + sqrtDisc) / (2 * a), (-b - sqrtDisc) / (2 * a));

        // Velocity when it reaches yTarget
        float v1 = vy - g * t0;

        // Now compute time to fall from yTarget to yMin
        float a2 = -0.5f * g;
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
        var calculatedVelocity = GetVelocity(startPos, targetPos, ShotControlMode.UseFlightTime, value);
        // Assuming Z=0 is the net position for simplicity
        var height = GetHeightAtSpecificZPosition(startPos, calculatedVelocity, 0);
        Debug.Log("GetHeightAtSpecificZPosition > " + height);
        return height;
    }

    public void SetVelocity(Vector3 initialVelocity)
    {
        rb.linearVelocity = initialVelocity; // Directly set Rigidbody's velocity

        // If you are setting spin manually, you can do it here:
        // rb.angularVelocity = Vector3.forward * 10f; // Example: apply some topspin

        isMoving = true;
        hasBounced = false; // Reset bounce flag
        float initialSpeedMPS = rb.linearVelocity.magnitude;
        Debug.Log($"Initial Shot Speed: {initialSpeedMPS:F2} m/s ({initialSpeedMPS * 3.6f:F2} km/h)");
        initialShotVelocity = initialVelocity; // Store for potential debug/trajectory display
    }

    public Vector3 GetVelocity(
        Vector3 startPos,
        Vector3 targetPos,
        ShotControlMode mode,
        float value // either flightTime or arcHeight depending on mode
    )
    {
        Vector3 displacement = targetPos - startPos;
        Vector3 displacementXZ = new Vector3(displacement.x, 0f, displacement.z);
        float g = -Physics.gravity.y; // Get the magnitude of gravity from Unity's settings
        float flightTime;
        float vy;
        Vector3 velocityXZ;

        if (mode == ShotControlMode.UseFlightTime)
        {
            flightTime = value;

            if (flightTime <= 0)
            {
                Debug.LogError("Flight time must be positive.");
                return Vector3.zero;
            }

            velocityXZ = new Vector3(
                displacementXZ.x / flightTime,
                0f,
                displacementXZ.z / flightTime
            );

            // kinematic equation: target.y = start.y + vy*t - 0.5*g*t*t
            // vy = (target.y - start.y + 0.5*g*t*t) / t
            vy = (targetPos.y - startPos.y + 0.5f * g * flightTime * flightTime) / flightTime;
        }
        else // UseArcHeight
        {
            float arcHeight = value;
            if (arcHeight < 0)
            {
                Debug.LogError("Arc height must be non-negative.");
                return Vector3.zero;
            }

            // Calculate peak height relative to the higher of start or target Y
            float initialY = startPos.y;
            float finalY = targetPos.y;
            float requiredPeakHeight = Mathf.Max(initialY, finalY) + arcHeight;

            // Calculate initial vertical velocity needed to reach peak height from initialY
            // v_f^2 = v_i^2 + 2*a*dy  => 0 = v_i^2 - 2*g*(requiredPeakHeight - initialY)
            // v_i^2 = 2*g*(requiredPeakHeight - initialY)
            vy = Mathf.Sqrt(2f * g * (requiredPeakHeight - initialY));

            // If the target is higher than the initial position, the initial vertical velocity might need adjustment
            // This is a common simplification that might need refinement for more complex trajectory calculations
            // where peak isn't necessarily directly above initial or final.
            // For a symmetrical arc from initial to peak, timeToPeak = vy / g
            float timeToPeak = vy / g;

            // Time to fall from peak to target.y
            // dy = 0.5 * g * t^2 => t = sqrt(2*dy/g)
            float timeToFallFromPeak = Mathf.Sqrt(2f * (requiredPeakHeight - finalY) / g);

            flightTime = timeToPeak + timeToFallFromPeak;

            if (flightTime <= 0)
            {
                Debug.LogError("Calculated flight time is not positive for arc height mode.");
                return Vector3.zero;
            }

            velocityXZ = new Vector3(
                displacementXZ.x / flightTime,
                0f,
                displacementXZ.z / flightTime
            );
        }

        return velocityXZ + (Vector3.up * vy);
    }

    // Inside your TennisBallLauncher.cs, perhaps in the ShootTo method,
    // or a public method you call immediately after ShootTo.
    // A more robust prediction method that can simulate bounces
    public TrajectoryPoint[] PredictTrajectory(
        Vector3 startPos,
        Vector3 startVel,
        Vector3 startAngularVel,
        float predictionTime,
        float timeStep = 0.02f
    )
    {
        List<TrajectoryPoint> points = new List<TrajectoryPoint>();

        Vector3 simPos = startPos;
        Vector3 simVel = startVel;
        Vector3 simAngularVel = startAngularVel;
        float courtY = 0f; // Assuming your court ground is at Y = 0

        // Get the ball's own material properties for prediction
        float ballBounciness = ballPhysicMaterialInstance.bounciness;
        float ballDynamicFriction = ballPhysicMaterialInstance.dynamicFriction;
        PhysicsMaterialCombine ballBounceCombine = ballPhysicMaterialInstance.bounceCombine;
        PhysicsMaterialCombine ballFrictionCombine = ballPhysicMaterialInstance.frictionCombine;

        // Get ground material properties for prediction (use defaults if not assigned)
        float groundBounciness = (groundPhysicMaterial != null) ? groundPhysicMaterial.bounciness : 1.0f; // Default to perfectly bouncy if no ground material
        float groundDynamicFriction = (groundPhysicMaterial != null) ? groundPhysicMaterial.dynamicFriction : 1.0f; // Default to no friction
        PhysicsMaterialCombine groundBounceCombine = (groundPhysicMaterial != null) ? groundPhysicMaterial.bounceCombine : PhysicsMaterialCombine.Average;
        PhysicsMaterialCombine groundFrictionCombine = (groundPhysicMaterial != null) ? groundPhysicMaterial.frictionCombine : PhysicsMaterialCombine.Average;


        // Calculate the EFFECTIVE combined properties for bounce based on Unity's rules.
        // We'll take the 'combined' mode from the ball's material, as Unity tends to prioritize the more complex one,
        // but for simplicity, we'll just implement the common 'Multiply' here directly for clarity.
        // If you need other modes, you'd add more switch cases.

        float effectiveBounciness = 1.0f;
        float effectiveDynamicFriction = 1.0f;

        // Simplify for common 'Multiply' case
        // If you use other combine modes (Average, Min, Max), you'll need to implement those here too.
        if (ballBounceCombine == PhysicsMaterialCombine.Multiply && groundBounceCombine == PhysicsMaterialCombine.Multiply)
        {
             effectiveBounciness = ballBounciness * groundBounciness;
        }
        else // Fallback for other modes or if they differ (often Average is used)
        {
            // You might need to decide which combine mode to prioritize if they differ
            // For example, if one is Multiply and the other is Average, Unity has rules.
            // A common simplified fallback is Average:
            effectiveBounciness = (ballBounciness + groundBounciness) / 2f;
        }


        if (ballFrictionCombine == PhysicsMaterialCombine.Multiply && groundFrictionCombine == PhysicsMaterialCombine.Multiply)
        {
            effectiveDynamicFriction = ballDynamicFriction * groundDynamicFriction;
        }
        else
        {
            effectiveDynamicFriction = (ballDynamicFriction + groundDynamicFriction) / 2f;
        }


        // Clamp values to ensure they stay within reasonable physical bounds (0 to 1)
        effectiveBounciness = Mathf.Clamp01(effectiveBounciness);
        effectiveDynamicFriction = Mathf.Clamp01(effectiveDynamicFriction);


        for (float t = 0f; t < predictionTime; t += timeStep)
        {
            Vector3 prevSimPos = simPos;

            // Apply gravity
            Vector3 acceleration = Physics.gravity;

            // Apply drag (assuming linear drag model for simplicity)
            acceleration -= simVel * dragCoefficient;

            // Apply Magnus effect (simple approximation)
            Vector3 magnusForce = Vector3.Cross(simAngularVel.normalized, simVel.normalized) * simAngularVel.magnitude * spinForceMultiplier;
            acceleration += magnusForce / rb.mass; // Divide by mass to get acceleration

            simVel += acceleration * timeStep;
            simPos += simVel * timeStep;

            bool bounced = false;
            // Simplified ground collision detection (assuming flat ground at Y=0)
            if (simPos.y < courtY + ballRadius && prevSimPos.y >= courtY + ballRadius)
            {
                simPos.y = courtY + ballRadius; // Correct position to avoid sinking
                bounced = true;

                // Calculate normal and tangential components of velocity for bounce
                Vector3 normalVelocity = Vector3.Project(simVel, Vector3.up);
                Vector3 tangentialVelocity = simVel - normalVelocity;

                // Apply EFFECTIVE bounciness for normal velocity
                normalVelocity *= -effectiveBounciness;

                // Apply EFFECTIVE dynamic friction for tangential velocity
                tangentialVelocity *= (1f - effectiveDynamicFriction);

                simVel = normalVelocity + tangentialVelocity;

                // Simple spin damping on bounce
                simAngularVel *= 0.5f; // Dampen angular velocity (you might need a more complex model)
            }

            points.Add(new TrajectoryPoint
            {
                position = simPos,
                velocity = simVel,
                angularVelocity = simAngularVel,
                bounced = bounced,
                time = t
            });
        }
        return points.ToArray();
    }
    public Vector3 DrawPreBounceTrajectoryAndTrackMaxHeight(
    Vector3 startPos,
    Vector3 startVel,
    Vector3 startAngularVel,
    float predictionTime,
    float timeStep,
    LineRenderer lineRendererToDraw)
    {
        var points = PredictTrajectory(startPos, startVel, startAngularVel, predictionTime, timeStep);

        List<Vector3> preBouncePoints = new List<Vector3>();
        bool foundBounce = false;
        int bounceIndex = -1;

        for (int i = 0; i < points.Length; i++)
        {
            var point = points[i];
            preBouncePoints.Add(point.position);
            if (point.bounced)
            {
                foundBounce = true;
                bounceIndex = i;
                break;
            }
        }

        // Draw only the pre-bounce path
        if (lineRendererToDraw != null)
        {
            lineRendererToDraw.positionCount = preBouncePoints.Count;
            lineRendererToDraw.SetPositions(preBouncePoints.ToArray());
        }

        // Track max height after bounce
        Vector3 maxHeightAfterBounce = new Vector3();

        if (foundBounce && bounceIndex + 1 < points.Length)
        {
            for (int i = bounceIndex + 1; i < points.Length; i++)
            {
                Vector3 y = points[i].position;
                if (y.y > maxHeightAfterBounce.y)
                    maxHeightAfterBounce = y;

                // Stop early if we passed the peak
                if (points[i].velocity.y < 0 && points[i].position.y < points[bounceIndex].position.y)
                    break;
            }

            Debug.Log($"Max height after first bounce: {maxHeightAfterBounce:F3} meters");
            return maxHeightAfterBounce;
        }

        Debug.Log("No bounce detected or no post-bounce points.");
        return Vector3.zero;
    }

    public TrajectoryPoint[] GetPredictedBounceVelocity(Vector3 startPos, Vector3 initialLaunchVelocity, Vector3 initialLaunchAngularVelocity)
    {
        // Predict the entire trajectory from the start of the shot
        // Predict for a reasonable amount of time, e.g., 5 seconds, to ensure a bounce can be found.
        // The timeStep should be small for accuracy.
        float predictionTime = 5.0f; // Max time to simulate
        float timeStep = 0.02f;     // Simulation step (matching FixedUpdate or smaller)

        TrajectoryPoint[] predictedPath = PredictTrajectory(
            startPos,
            initialLaunchVelocity,
            initialLaunchAngularVelocity,
            predictionTime,
            timeStep
        );

        // Iterate through the predicted points to find the first bounce
        TrajectoryPoint? firstBouncePoint = null;
        foreach (var point in predictedPath)
        {
            if (point.bounced)
            {
                firstBouncePoint = point;
                break; // Found the first bounce, stop searching
            }
        }

        if (firstBouncePoint.HasValue)
        {
            // Now you have the predicted velocity right after the first bounce!
            Vector3 velocityAfterFirstBounce = firstBouncePoint.Value.velocity;
            Vector3 positionAtFirstBounce = firstBouncePoint.Value.position;
            float timeAtFirstBounce = firstBouncePoint.Value.time;

            Debug.Log($"Predicted First Bounce at: {positionAtFirstBounce} (Time: {timeAtFirstBounce:F2}s)");
            Debug.Log($"Predicted Velocity AFTER First Bounce: {velocityAfterFirstBounce}");

            // You could return this value, store it, or use it immediately.
            // For example, an AI could use this to anticipate future movement.
        }
        else
        {
            Debug.Log("No bounce predicted within the specified prediction time.");
        }
        return predictedPath;
    }
    /// <summary>
    /// Shoots the ball towards a target using Unity's physics.
    /// </summary>
    public void ShootTo(
        Vector3 startPos,
        Vector3 targetPos,
        ShotControlMode mode,
        float value, // either flightTime or arcHeight depending on mode
        Vector3 initialSpinAngularVelocity = default(Vector3) // New parameter for initial spin
    )
    {
        rb.position = startPos; // Set Rigidbody's position
        transform.position = startPos; // Ensure transform is also aligned

        rb.isKinematic = false;
        rb.useGravity = true;
        GetComponent<SphereCollider>().enabled = true;
        Vector3 calculatedVelocity = GetVelocity(startPos, targetPos, mode, value);
        rb.linearVelocity = calculatedVelocity; // Set Rigidbody's initial velocity
        initialShotVelocity = calculatedVelocity; // Store it

        rb.angularVelocity = initialSpinAngularVelocity; // Apply initial spin

        isMoving = true;
        hasBounced = false; // Reset bounce flag
        float initialSpeedMPS = rb.linearVelocity.magnitude;
        Debug.Log($"Initial Shot Speed: {initialSpeedMPS:F2} m/s ({initialSpeedMPS * 3.6f:F2} km/h) | Initial Spin: {rb.angularVelocity.magnitude:F2} rad/s");
        oldYPosition = transform.position.y;
        start = transform.position;
        //debugHeight = true;
    }

    // New: Handle collisions using Unity's OnCollisionEnter
    void OnCollisionEnter(Collision collision)
    {
        // Unity's Rigidbody automatically handles the bounce (reflection and damping)
        // and friction based on its properties and the PhysicMaterial.

        Debug.Log($"Collision with: {collision.gameObject.name} on layer: {LayerMask.LayerToName(collision.gameObject.layer)}");

        // Check if the ball is moving, otherwise no bounce logic is needed
        if (!isMoving) return;

        // Check for ground collision (assuming ground layer is assigned)
        if (collision.gameObject.CompareTag("Ground") || collision.gameObject.layer == LayerMask.NameToLayer("Ground")) // Using tag or layer
        {
            if (!hasBounced)
            {
                oldYPosition = transform.position.y;
                start = transform.position;
                debugHeight = true;
                hasBounced = true;
                Debug.Log("Bounce (Ground) > " + (swipeInput != null ? (Time.time - swipeInput.startTime).ToString() : "N/A - SwipeInput not linked"));
                OnBounce?.Invoke();
            }
        }
        else if (collision.gameObject.CompareTag("Net") || collision.gameObject.layer == LayerMask.NameToLayer("Net"))
        {
            Debug.Log("Collision with Net!");
            // You can add specific effects here, e.g., sound for net hit
        }
        else // Other collisions (walls, etc.)
        {
            Debug.Log($"Collision with other object: {collision.gameObject.name}");
        }

        // After a collision, re-check if the ball should stop
        CheckStop();
    }

    // New: Handle triggers (if you have trigger zones, e.g., out-of-bounds)
    void OnTriggerEnter(Collider other)
    {
        // Example: If ball goes out of bounds
        if (other.CompareTag("OutOfBounds"))
        {
            Debug.Log("Ball went out of bounds!");
            StopBall(); // Stop the ball or reset it
        }
    }


    private void CheckStop()
    {
/*        // Check if the ball's velocity and angular velocity are very low
        // This is a more robust way to check for stopping than just `isGrounded`
        if (rb.linearVelocity.magnitude < 0.1f && rb.angularVelocity.magnitude < 0.1f)
        {
            StopBall();
        }*/
    }

    private void StopBall()
    {
        isMoving = false;
        rb.linearVelocity = Vector3.zero;       // Stop linear movement
        rb.angularVelocity = Vector3.zero; // Stop rotation
        OnStop?.Invoke();
        Debug.Log("Ball stopped.");
    }

    public bool IsMoving() => isMoving;
    public Vector3 GetVelocity() => rb.linearVelocity; // Get current velocity from Rigidbody

    public void ResetBall(Vector3 position)
    {
        rb.position = position;
        transform.position = position; // Keep transform aligned
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        isMoving = false;
        hasBounced = false;
    }

    void OnDrawGizmos()
    {
        if (!Application.isPlaying || !isMoving || rb == null) return;

        // Velocity vector
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + rb.linearVelocity.normalized * 2f);
        Gizmos.DrawSphere(transform.position + rb.linearVelocity.normalized * 2f, 0.1f);

        // Angular Velocity vector (spin axis)
        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, transform.position + rb.angularVelocity.normalized * 1f);
        Gizmos.DrawSphere(transform.position + rb.angularVelocity.normalized * 1f, 0.05f);

        // Magnus force direction (if angular velocity and velocity are not zero)
        if (rb.angularVelocity.magnitude > 0.001f && rb.linearVelocity.magnitude > 0.001f)
        {
            Gizmos.color = Color.blue;
            Vector3 magnusDir = Vector3.Cross(rb.angularVelocity.normalized, rb.linearVelocity.normalized); // Normalized for direction
            Gizmos.DrawLine(transform.position, transform.position + magnusDir * 2f);
            Gizmos.DrawSphere(transform.position + magnusDir * 2f, 0.1f);
        }
    }
}