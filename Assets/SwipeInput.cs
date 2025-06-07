using System;
using UnityEngine;
using UnityEngine.Events; // You're using this, but not in the code provided. Keep if needed elsewhere.

public class SwipeInput : MonoBehaviour
{
    [Header("Swipe Detection Settings")]
    public float minSwipeDistance = 50f; // Minimum pixels to qualify as swipe
    public float maxSwipeTime = 1f;      // Max duration to consider a swipe
    public float minNormalizedSwipeSpeed = 100f; // Minimum raw px/s for 0 strength
    public float maxNormalizedSwipeSpeed = 1000f; // Maximum raw px/s for 1 strength

    [Header("References")]
    public LayerMask groundMask;

    // These should ideally be set up in the Inspector as transforms or just passed directly to ShootTo
    public Transform playerHitPosition; // The world position where the player 'hits' the ball (e.g., racket position)
    public Transform courtTargetPosition; // The world position on the court the player is aiming for (e.g., target reticle)


    private Vector2 startTouchPos;
    public float startTime;
    private Vector3 initialBallWorldPosition; // World position where the ball starts its trajectory
    private Vector3 finalTargetWorldPosition; // World position on the court where the player aimed

    // The event to notify when a shot is ready to be fired with all calculated parameters
    public Action<Vector3 /*startPos*/, Vector3 /*targetPos*/, float /*normalizedSwipeStrength*/> OnShotReady;


    void Awake()
    {
        if (playerHitPosition == null)
            Debug.LogWarning("Player Hit Position Transform not set. Ball will start at (0,0,0) in world space.");
        if (courtTargetPosition == null)
            Debug.LogWarning("Court Target Position Transform not set. Target will be (0,0,0) in world space.");
    }

    void Update()
    {
#if UNITY_EDITOR
        HandleMouseInput();
#else
        HandleTouchInput();
#endif
    }

    private void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            startTime = Time.time;
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit, 100, groundMask))
            {
                initialBallWorldPosition = new Vector3(hit.point.x, 0, hit.point.z); // Aim at the ground
            }
        }

        if (Input.GetMouseButtonUp(0))
        {
            float swipeTime = Time.time - startTime;
            Debug.Log("Mouse Up > " + Time.time);

            // Re-evaluate final target position on mouse up for precision
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit, 100, groundMask))
            {
                finalTargetWorldPosition = new Vector3(hit.point.x, 0, hit.point.z);
            }
            // If no ground hit, use the one from MouseDown or a default fallback
            // (consider making this more robust for off-court targeting)

            ProcessSwipe(swipeTime);
        }
    }

    private void HandleTouchInput()
    {
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);

            if (touch.phase == TouchPhase.Began)
            {
                startTouchPos = touch.position;
                startTime = Time.time;

                initialBallWorldPosition = playerHitPosition != null ? playerHitPosition.position : Vector3.zero;

                // For touch, determining initial world target might be tricky.
                // You might defer it to ProcessSwipe or use a fixed target,
                // or raycast at the beginning of the swipe.
                // For simplicity here, we'll use the final raycast on touch.ended.
            }
            else if (touch.phase == TouchPhase.Ended)
            {
                float swipeTime = Time.time - startTime;
                ProcessSwipe(swipeTime);
            }
        }
    }

    private void ProcessSwipe(float swipeTime)
    {
        OnShotReady?.Invoke(initialBallWorldPosition, finalTargetWorldPosition, swipeTime);
    }
}