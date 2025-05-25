using UnityEngine;

public class TennisBallLauncher : MonoBehaviour
{
    public CustomTennisBallRigidbody ballRigidbodyPrefab;
    public Transform launchPoint;
    public float servePowerMultiplier = 2f;
    public float flatPowerMultiplier = 1f;
    public float topspinLiftMultiplier = 0.5f;
    public float topspinSpinMultiplier = 1f;
    public float sliceLiftMultiplier = 0.2f;
    public float sliceSpinMultiplier = -1f; // Opposite spin for slice
    public float lobLiftMultiplier = 1.5f;
    public float volleyPowerMultiplier = 0.8f;

    private CustomTennisBallRigidbody currentBall;

    void Start()
    {
        if (ballRigidbodyPrefab == null)
        {
            Debug.LogError("Ball Rigidbody Prefab not assigned!");
            enabled = false;
            return;
        }

        if (launchPoint == null)
        {
            launchPoint = transform;
        }

        SpawnBall();
    }

    public void SpawnBall()
    {
        if (currentBall != null)
        {
            Destroy(currentBall.gameObject);
        }
        GameObject ballObject = Instantiate(ballRigidbodyPrefab.gameObject, launchPoint.position, Quaternion.identity);
        currentBall = ballObject.GetComponent<CustomTennisBallRigidbody>();
        if (currentBall == null)
        {
            Debug.LogError("Prefab does not have CustomTennisBallRigidbody component!");
            enabled = false;
        }
    }

    public void Serve(Vector3 direction, ShotData power)
    {
        if (currentBall != null && !currentBall.IsMoving())
        {

            currentBall.Launch(direction, power);
        }
    }

    public void FlatShot(Vector3 direction, float power)
    {
        if (currentBall != null && !currentBall.IsMoving())
        {
            ShotData flat = new ShotData
            {
                power = power * flatPowerMultiplier,
                lift = 0f,
                spin = 0f,
                drag = 0.01f
            };
            currentBall.Launch(direction, flat);
        }
    }

    public void Topspin(Vector3 direction, float power)
    {
        if (currentBall != null && !currentBall.IsMoving())
        {
            ShotData topspinData = new ShotData
            {
                power = power * flatPowerMultiplier, // Base power
                lift = power * topspinLiftMultiplier, // Significant upward lift
                spin = power * topspinSpinMultiplier, // Strong forward spin (positive value)
                drag = 0.015f // Might increase drag slightly with spin
            };
            currentBall.Launch(direction, topspinData);
        }
    }

    public void Slice(Vector3 direction, float power)
    {
        if (currentBall != null && !currentBall.IsMoving())
        {
            ShotData sliceData = new ShotData
            {
                power = power * flatPowerMultiplier * 0.9f, // Slightly less power than flat
                lift = power * sliceLiftMultiplier, // Small upward lift or even slight downward
                spin = power * sliceSpinMultiplier, // Strong backward/sideways spin (negative value)
                drag = 0.015f // Might increase drag slightly with spin
            };
            currentBall.Launch(direction, sliceData);
        }
    }

    public void Lob(Vector3 direction, float power)
    {
        if (currentBall != null && !currentBall.IsMoving())
        {
            ShotData lobData = new ShotData
            {
                power = power * 0.7f, // Moderate power
                lift = power * lobLiftMultiplier, // High upward lift
                spin = 5f, // Small amount of spin for stability
                drag = 0.01f
            };
            currentBall.Launch(direction, lobData);
        }
    }

    public void Volley(Vector3 direction, float power)
    {
        if (currentBall != null && !currentBall.IsMoving())
        {
            ShotData volleyData = new ShotData
            {
                power = power * volleyPowerMultiplier, // Less power for close-range volley
                lift = 0.05f, // Minimal lift
                spin = 2f,   // Small amount of spin for control
                drag = 0.005f // Less drag at closer range
            };
            currentBall.Launch(direction, volleyData);
        }
    }

    public void ResetBall()
    {
        if (currentBall != null)
        {
            SpawnBall();
        }
    }

    public bool IsBallMoving()
    {
        return currentBall != null && currentBall.IsMoving();
    }

    public Vector3 GetBallVelocity()
    {
        return currentBall != null ? currentBall.GetVelocity() : Vector3.zero;
    }
}