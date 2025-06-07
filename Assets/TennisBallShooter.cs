using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class TennisBallShooter : MonoBehaviour
{
    public float shotSpeedKmh = 120f;         // Ball speed in km/h
    public Vector3 spinRPM = Vector3.zero;    // Spin vector: topspin/slice/sidespin
    public float netClearanceHeight = 1.0f;   // Optional: desired net clearance height
    public float magnusForceFactor = 0.00005f;

    public Rigidbody rb;
    private Vector3 spinRad;
    bool isLaunched = false;
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        if (!isLaunched)
        {
            return;
        }
        ApplyMagnusForce();
    }

    public void LaunchShot(Vector3 startPos, Vector3 targetPos)
    {
        // Convert speed to m/s
        float speedMps = shotSpeedKmh / 3.6f;

        // Convert spin from RPM to rad/s
        spinRad = spinRPM * Mathf.Deg2Rad / 60f;

        // Estimate time of flight
        float distance = Vector3.Distance(startPos, targetPos);
        float flightTime = distance / speedMps;

        // Compensate aim for Magnus effect
        Vector3 dir = (targetPos - startPos).normalized;
        Vector3 compensationDir = Vector3.Cross(spinRad.normalized, dir);
        Vector3 compensatedTarget = targetPos + compensationDir * 0.5f; // tune factor (0.5f) for realism

        // Recalculate direction with compensation
        Vector3 adjustedDir = (compensatedTarget - startPos).normalized;

        // Add vertical angle for arc
        float heightOffset = Mathf.Max(netClearanceHeight - startPos.y, 0f);
        Vector3 velocity = adjustedDir * speedMps;
        velocity.y += heightOffset / flightTime;


        // Apply to rigidbody
        rb.isKinematic = false;
        rb.useGravity = true;
        rb.linearVelocity = velocity;
    }

    void ApplyMagnusForce()
    {
        if (rb.linearVelocity.magnitude < 0.1f) return;

        Vector3 liftDir = Vector3.Cross(spinRad, rb.linearVelocity.normalized);
        Vector3 magnusForce = liftDir.normalized * spinRad.magnitude * rb.linearVelocity.magnitude * magnusForceFactor;

        rb.AddForce(magnusForce, ForceMode.Force);
    }
}