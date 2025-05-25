using UnityEngine;

public class TennisShotController : MonoBehaviour
{
    public Rigidbody ballRb;
    public Transform ball;
    public Transform opponentCourtCenter;

    public float maxSwipeLength = 500f;       // max screen swipe length (pixels)
    public float maxShotDistance = 8f;        // max shot distance in meters (court depth)

    public float minSwipeTime = 0.1f;          // shortest swipe duration (seconds)
    public float maxSwipeTime = 0.6f;          // longest swipe duration (seconds)
    public float minFlightTime = 0.5f;         // flight time for fastest shot
    public float maxFlightTime = 1.5f;         // flight time for lob

    public float minSpin = 0f;
    public float maxSpin = 300f;

    public float gravity = 9.81f;

    public float playerPower = 1f;             // power multiplier (1 = normal)

    private Vector2 swipeStartPos;
    private float swipeStartTime;

    void Update()
    {
        // Start swipe
        if (Input.GetMouseButtonDown(0))
        {
            swipeStartPos = Input.mousePosition;
            swipeStartTime = Time.time;
        }

        // End swipe and launch shot
        if (Input.GetMouseButtonUp(0))
        {
            Vector2 swipeEndPos = Input.mousePosition;
            float swipeDuration = Time.time - swipeStartTime;
            Vector2 swipeVector = swipeEndPos - swipeStartPos;

            LaunchShot(swipeStartPos, swipeVector, swipeDuration);
        }
    }

    void LaunchShot(Vector2 swipeStart, Vector2 swipeVector, float swipeDuration)
    {
        Rigidbody ballObject = Instantiate(ballRb.gameObject, ball.position, Quaternion.identity).GetComponent<Rigidbody>();

        // --- 1. Calculate target point on opponent court ---
        ballObject.isKinematic = false;
        // Normalize swipe length and clamp
        float swipeLength = swipeVector.magnitude;
        float depthFactor = Mathf.Clamp01(swipeLength / maxSwipeLength);

        // Calculate swipe direction in world space
        Vector3 worldSwipeStart = Camera.main.ScreenToWorldPoint(new Vector3(swipeStart.x, swipeStart.y, Camera.main.nearClipPlane + 1f));
        Vector3 worldSwipeEnd = Camera.main.ScreenToWorldPoint(new Vector3(swipeStart.x + swipeVector.x, swipeStart.y + swipeVector.y, Camera.main.nearClipPlane + 1f));
        Vector3 swipeDirection = (worldSwipeEnd - worldSwipeStart).normalized;

        // Calculate target position on court
        Vector3 target = opponentCourtCenter.position + swipeDirection * depthFactor * maxShotDistance;

        // --- 2. Normalize swipe duration ---
        float t = Mathf.InverseLerp(minSwipeTime, maxSwipeTime, swipeDuration);

        // --- 3. Calculate flight time modified by player power ---
        float flightTime = Mathf.Lerp(minFlightTime, maxFlightTime, t) / playerPower;

        // --- 4. Calculate initial velocity for projectile motion ---

        Vector3 startPos = ball.position;
        Vector3 displacement = target - startPos;

        Vector3 horizontalDisplacement = new Vector3(displacement.x, 0f, displacement.z);
        float verticalDisplacement = displacement.y;

        Vector3 v0_horizontal = horizontalDisplacement / flightTime;
        float v0_vertical = (verticalDisplacement + 0.5f * gravity * flightTime * flightTime) / flightTime;

        Vector3 initialVelocity = v0_horizontal + Vector3.up * v0_vertical;

        // --- 5. Launch ball ---
        ballObject.angularVelocity = initialVelocity;

        // --- 6. Apply spin based on swipe duration ---
        float spinAmount = Mathf.Lerp(minSpin, maxSpin, t);
        float spinDirection = Mathf.Lerp(1f, -1f, t); // topspin → backspin

        ballObject.AddTorque(ball.transform.right * spinAmount * spinDirection);
    }
}
