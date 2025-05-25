using UnityEngine;

public class SwipeLauncher : MonoBehaviour
{
    public TennisBallLauncher ballLauncher;
    public Camera mainCamera; // Assign your main camera

    [Header("Serve Settings")]
    public float serveSwipeThreshold = 80f;
    public float serveMaxSwipeTime = 0.7f;
    public float servePowerMultiplier = 0.15f;
    public float serveMaxPower = 25f;
    public float serveLiftMultiplier = 0.8f; // Increased lift for serves
    public float serveSpinMultiplier = 0.3f; // Slight spin for serve control
    public Vector3 serveDirectionOffset = new Vector3(0f, 0.5f, 1f); // Offset from forward for serve direction
    public float serveMaxVerticalAngle = 60f; // Limit upward serve angle

    [Header("Regular Shot Settings")]
    public float swipeThreshold = 50f; // Minimum swipe distance in pixels
    public float maxSwipeTime = 0.5f; // Maximum time for a valid swipe
    public float powerMultiplier = 0.1f; // Adjust for swipe speed to power conversion
    public float maxPower = 20f; // Maximum launch power
    public float spinMultiplier = 0.2f; // Adjust for sideways swipe to spin conversion
    public float maxSpin = 20f; // Maximum spin
    public float lobVerticalThreshold = 0.8f; // Normalized vertical swipe to trigger lob
    public float topspinVerticalThreshold = 0.3f; // Normalized vertical swipe for topspin

    private Vector2 startTouchPosition;
    private float startTime;
    private bool isSwiping = false;
    private Vector3 swipeStartWorldPosition;
    private Vector3 swipeEndWorldPosition;
    public LayerMask groundMask;
    private bool isServing = false;

    public void EnableServeMode(Vector3 servePosition)
    {
        isServing = true;
        if (ballLauncher != null)
        {
            ballLauncher.transform.position = servePosition;
            ballLauncher.ResetBall();
        }
        else
        {
            Debug.LogError("Ball Launcher is null!");
            isServing = false;
        }
    }

    public void DisableServeMode()
    {
        isServing = false;
    }

    void Update()
    {
        if (mainCamera == null)
        {
            Debug.LogError("Main Camera not assigned!");
            enabled = false;
            return;
        }

        // Handle touch input (for mobile)
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);

            switch (touch.phase)
            {
                case TouchPhase.Began:
                    startTouchPosition = touch.position;
                    startTime = Time.time;
                    isSwiping = true;
                    Ray startRay = mainCamera.ScreenPointToRay(startTouchPosition);
                    RaycastHit startHit;
                    if (Physics.Raycast(startRay, out startHit, 100, groundMask))
                    {
                        swipeStartWorldPosition = startHit.point;
                    }
                    else
                    {
                        swipeStartWorldPosition = GetWorldPositionOnPlane(startTouchPosition, Vector3.up, transform.position.y);
                    }
                    break;
                case TouchPhase.Ended:
                    if (isSwiping)
                    {
                        Vector2 endTouchPosition = touch.position;
                        float swipeTime = Time.time - startTime;
                        float swipeDistance = (endTouchPosition - startTouchPosition).magnitude;

                        Ray endRay = mainCamera.ScreenPointToRay(endTouchPosition);
                        RaycastHit endHit;
                        if (Physics.Raycast(endRay, out endHit, 100, groundMask))
                        {
                            swipeEndWorldPosition = endHit.point;
                        }
                        else
                        {
                            swipeEndWorldPosition = GetWorldPositionOnPlane(endTouchPosition, Vector3.up, transform.position.y);
                        }

                        Vector3 swipeDirectionWorld = swipeEndWorldPosition - swipeStartWorldPosition;

                        if (isServing)
                        {
                            if (swipeTime < serveMaxSwipeTime && swipeDistance > serveSwipeThreshold)
                            {
                                ServeBall(swipeDirectionWorld, swipeTime, swipeDistance);
                            }
                            isServing = false; // Exit serve mode after the serve
                        }
                        else
                        {
                            if (swipeTime < maxSwipeTime && swipeDistance > swipeThreshold)
                            {
                                LaunchBall(swipeDirectionWorld, swipeTime, swipeDistance);
                            }
                        }
                        isSwiping = false;
                    }
                    break;
                case TouchPhase.Canceled:
                    isSwiping = false;
                    break;
            }
        }
        // Handle mouse input (for testing in editor)
        else if (Input.GetMouseButtonDown(0))
        {
            startTime = Time.time;
            startTouchPosition = Input.mousePosition;
            Ray startRay = mainCamera.ScreenPointToRay(startTouchPosition);
            RaycastHit startHit;
            if (Physics.Raycast(startRay, out startHit, 100, groundMask))
            {
                swipeStartWorldPosition = startHit.point;
            }
            else
            {
                swipeStartWorldPosition = GetWorldPositionOnPlane(startTouchPosition, Vector3.up, transform.position.y);
            }
            isSwiping = true;
        }
        else if (Input.GetMouseButtonUp(0) && isSwiping)
        {
            Vector2 endTouchPosition = Input.mousePosition;
            float swipeTime = Time.time - startTime;
            float swipeDistance = (endTouchPosition - startTouchPosition).magnitude;
            Ray endRay = mainCamera.ScreenPointToRay(endTouchPosition);
            RaycastHit endHit;
            if (Physics.Raycast(endRay, out endHit, 100, groundMask))
            {
                swipeEndWorldPosition = endHit.point;
            }
            else
            {
                swipeEndWorldPosition = GetWorldPositionOnPlane(endTouchPosition, Vector3.up, transform.position.y);
            }

            Vector3 swipeDirectionWorld = swipeEndWorldPosition - swipeStartWorldPosition;

            if (isServing)
            {
                if (swipeTime < serveMaxSwipeTime && swipeDistance > serveSwipeThreshold)
                {
                    ServeBall(swipeDirectionWorld, swipeTime, swipeDistance);
                }
                //isServing = false;
            }
            else
            {
                if (swipeTime < maxSwipeTime && swipeDistance > swipeThreshold)
                {
                    LaunchBall(swipeDirectionWorld, swipeTime, swipeDistance);
                }
            }
            isSwiping = false;
        }
    }

    Vector3 GetWorldPositionOnPlane(Vector2 screenPosition, Vector3 planeNormal, float planeY)
    {
        Ray ray = mainCamera.ScreenPointToRay(screenPosition);
        Plane plane = new Plane(planeNormal, new Vector3(0, planeY, 0));
        float enter;
        if (plane.Raycast(ray, out enter))
        {
            return ray.GetPoint(enter);
        }
        return Vector3.zero;
    }

    void ServeBall(Vector3 swipeDirectionWorld, float swipeTime, float swipeDistance)
    {
        //if (ballLauncher == null || ballLauncher.IsBallMoving()) return;
        ballLauncher.SpawnBall();

        Vector3 normalizedSwipe = swipeDirectionWorld.normalized;
        Vector3 serveDirection = normalizedSwipe + serveDirectionOffset.normalized * 0.3f;
        serveDirection.Normalize();

        // Limit vertical angle for serve
        if (Vector3.Dot(serveDirection, Vector3.up) > Mathf.Sin(serveMaxVerticalAngle * Mathf.Deg2Rad))
        {
            serveDirection = Vector3.ProjectOnPlane(serveDirection, Vector3.right).normalized;
            serveDirection += Vector3.up * Mathf.Sin(serveMaxVerticalAngle * Mathf.Deg2Rad);
            serveDirection.Normalize();
        }

        float swipeSpeed = swipeDirectionWorld.magnitude / swipeTime;
        float power = Mathf.Clamp(swipeSpeed * servePowerMultiplier, 0f, serveMaxPower);
        float verticalSwipeAmount = Vector3.Dot(normalizedSwipe, Vector3.up);
        float horizontalSwipeAmount = Vector3.Dot(normalizedSwipe, Vector3.right);
        float forwardSwipeAmount = Vector3.Dot(normalizedSwipe, Vector3.forward); // Assuming forward is Z

        float lift = Mathf.Clamp01(verticalSwipeAmount + 0.5f) * serveLiftMultiplier * power / serveMaxPower;
        float spin = horizontalSwipeAmount * serveSpinMultiplier * power / serveMaxPower * 20f;

        ShotData serveData = new ShotData
        {
            power = power,
            lift = lift,
            spin = spin,
            drag = 0.01f
        };

        ballLauncher.Serve(serveDirection, serveData);
        Debug.Log($"Served with direction: {serveDirection}, power: {power:F2}, lift: {lift:F2}, spin: {spin:F2}");
    }

    void LaunchBall(Vector3 swipeDirectionWorld, float swipeTime, float swipeDistance)
    {
        if (ballLauncher == null || ballLauncher.IsBallMoving()) return;
        ballLauncher.SpawnBall();

        Vector3 launchDirection = swipeDirectionWorld.normalized;
        float swipeSpeed = swipeDirectionWorld.magnitude / swipeTime;
        float power = Mathf.Clamp(swipeSpeed * powerMultiplier, 0f, maxPower);
        float verticalSwipeAmount = Vector3.Dot(launchDirection, Vector3.up);
        float horizontalSwipeAmount = Vector3.Dot(launchDirection, Vector3.right);

        float spin = Mathf.Clamp(horizontalSwipeAmount * spinMultiplier, -maxSpin, maxSpin);

        if (verticalSwipeAmount > lobVerticalThreshold)
        {
            ballLauncher.Lob(launchDirection, power);
        }
        else if (verticalSwipeAmount > topspinVerticalThreshold)
        {
            ballLauncher.Topspin(launchDirection, power);
        }
        else if (Mathf.Abs(horizontalSwipeAmount) > 0.5f)
        {
            ballLauncher.Slice(launchDirection, power);
        }
        else
        {
            ballLauncher.FlatShot(launchDirection, power);
        }

        Debug.Log($"Launched with direction: {launchDirection}, power: {power:F2}, spin: {spin:F2}");
    }
}