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