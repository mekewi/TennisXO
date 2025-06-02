using System;
using UnityEngine;
using UnityEngine.Events;

public class SwipeInput : MonoBehaviour
{


    public float minSwipeDistance = 50f; // Minimum pixels to qualify as swipe
    public float maxSwipeTime = 1f;      // Max duration to consider a swipe
    public GameObject ball;      // Max duration to consider a swipe
    public Transform target;      // Max duration to consider a swipe

    private Vector2 startTouchPos;
    private Vector3 ballStartPosition;
    private Vector3 endPosition;
    public LayerMask groundMask;
    public float startTime;
    public Action<Vector3 /*directionToTarget*/, float /*swipeSpeed*/, float /*swipeDistance*/> OnSwipe;
    private void Awake()
    {
    }
    void Update()
    {
#if UNITY_EDITOR
        if (Input.GetMouseButtonDown(0))
        {
            startTouchPos = Input.mousePosition;
            startTime = Time.time;
            Debug.Log("Mouse Down > "+ startTime);
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, 100, groundMask))
            {
                ballStartPosition = new Vector3(hit.point.x, 0, hit.point.z);
            }
            //ballStartPosition = new Vector3(ball.position.x, 0, ball.position.z);
        }
        if (Input.GetMouseButton(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, 100, groundMask))
            {
                endPosition = new Vector3(hit.point.x, 0, hit.point.z);
            }
            //target.position = transform.position + endPosition;
        }
        if (Input.GetMouseButtonUp(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit,100, groundMask))
            {
                endPosition = new Vector3(hit.point.x, 0, hit.point.z);
            }
            Vector2 endTouchPos = Input.mousePosition;
            float swipeTime = Time.time - startTime;
            Debug.Log("Mouse Up > "+ Time.time);
            HandleSwipe(endTouchPos, swipeTime);
        }
#else
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);
            
            if (touch.phase == TouchPhase.Began)
            {
                startTouchPos = touch.position;
                startTime = Time.time;
            }
            else if (touch.phase == TouchPhase.Ended)
            {
                float swipeTime = Time.time - startTime;
                HandleSwipe(touch.position, swipeTime);
            }
        }
#endif
    }

    private void HandleSwipe(Vector2 endTouchPos, float swipeTime)
    {

        Vector2 swipeDelta = endTouchPos - startTouchPos;

/*        if (swipeDelta.magnitude < minSwipeDistance || swipeTime > maxSwipeTime)
            return;
*/
        float swipeDistance = swipeDelta.magnitude;
        float swipeSpeed = swipeDistance / swipeTime;

        Vector3 directionToTarget = (endPosition - ballStartPosition).normalized;
        var distance = (endPosition - ballStartPosition).magnitude;
        OnSwipe?.Invoke(directionToTarget, swipeTime, distance);

    }
}
