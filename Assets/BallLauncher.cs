using System.Collections.Generic;
using UnityEditor.Overlays;
using UnityEngine;

public class BallLauncher : MonoBehaviour
{

    public float playerPower = 15f; // This is player's max shot power
    public float maxSwipeDistance = 800f; // Max length in pixels (tune this)
    public float maxHeight = 5f; // Max lift for a slow swipe
    public float minHeight = 0.5f; // Low lift for fast swipe

    public GameObject ball;
    public SwipeInput swipeInput;
    public Transform target;
    public Transform center;
    public ShotControlMode shotMode;
    public SpinType spinType;
    public float offset;
    public float spinOffset;
    public float heightSpeed;
    public float speedBoost;
    public bool isstatic;

    private void Awake()
    {
        swipeInput.OnSwipe += HandleSwipe;
    }
    private void HandleSwipe(Vector3 directionToTarget, float swipeSpeed, float swipeDistance)
    {
        Debug.Log("HandleSwipe > swipeSpeed "+ swipeSpeed+ " +swipeDistance+ "+ swipeDistance + " Direction > "+ directionToTarget);
        var newBall = Instantiate(ball).GetComponent<CustomTennisBallRigidbody>();
        newBall.transform.position = ball.transform.position;
        offset = isstatic ? offset : swipeSpeed * heightSpeed;
        newBall.SetSpin(spinType, spinOffset); // 1.5 is spin strength
        newBall.ShootTo(newBall.transform.position, target.position, shotMode, offset);
        //newBall.ShootTo(newBall.transform.position, target.position, directionToTarget, offset);

    }

}