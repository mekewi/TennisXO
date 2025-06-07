using Mono.Cecil;
using System.Collections.Generic;
using UnityEditor.Overlays;
using UnityEngine;

public class BallLauncher : MonoBehaviour
{
    public GameObject ball;
    public SwipeInput swipeInput;
    public Transform target;
    public Transform center;
    public ShotControlMode shotMode;
    public SpinType spinType;
    public float minSpeed;
    public float maxSpeed;
    public float minHeight;
    public float maxHeight;
    public float maxTime;
    public float spinAmount;
    public float speedKMH;
    public float speedKMHFlixable;
    public float netHeight;
    public bool isstatic;
    private void Awake()
    {
        swipeInput.OnShotReady += HandleSwipe;
    }
    private void HandleSwipe(Vector3 startPos,Vector3 endPos, float swipeSpeed)
    {
        var ballRigidbody = Instantiate(ball).GetComponent<CustomTennisBallRigidbody>();
        var swipDeference = (endPos - startPos);
        ballRigidbody.transform.position = ball.transform.position;
        target.position = (new Vector3(0,0,0) + swipDeference.normalized) * swipDeference.magnitude;
        Vector3 currentStartPos = ballRigidbody.transform.position;
        var deference = target.position - currentStartPos;
        var dir = deference.normalized;
        var distance = deference.magnitude;
        Debug.Log("dir > "+ dir);
        Debug.Log("distance > " + distance);
        Vector3 currentTargetPos = target.position;
        var timeClamp = Mathf.Clamp(swipeSpeed, 0.1f, maxTime);
        shotMode = timeClamp < 1 ? ShotControlMode.UseFlightTime : ShotControlMode.UseArcHeight;
        timeClamp = timeClamp / maxTime;
        timeClamp = Mathf.Clamp01(timeClamp);
        Debug.Log("timeClamp > " + timeClamp + " < swipeSpeed > "+ swipeSpeed);
        var height = 0f;
        double timeInKHM;
        Debug.Log("IsBallWillHit Net > " + height);
        if (shotMode == ShotControlMode.UseFlightTime)
        {
            speedKMHFlixable = speedKMH;
            do
            {
                timeInKHM = distance / (speedKMHFlixable / 3.6f);
                Debug.Log("speed in sec > " + timeInKHM);
                height = ballRigidbody.IsBallWillHit(currentStartPos, currentTargetPos, netHeight, (float)timeInKHM);
                var initvelocity = ballRigidbody.GetVelocity(currentStartPos,
                currentTargetPos,
                ShotControlMode.UseFlightTime,
                (float)timeInKHM);
                var targetY = ballRigidbody.CorrectInitialVelocityY(currentStartPos, initvelocity, 0, netHeight);
                //var timeRest = ballRigidbody.CalculateExtraTimeToReachYMin(ballRigidbody.transform.position.y, initvelocity.y, targetY, netHeight);
                Debug.Log("CorrectInitialVelocityY ? " + targetY);
                //Debug.Log("CorrectInitialVelocityY timeRest? " + timeRest);
                //timeInKHM += timeRest;
                //ballRigidbody.SetVelocity(velocity);
                //  return;
                speedKMHFlixable--;
            } while (height < netHeight);
            speedKMHFlixable = Mathf.Lerp(minSpeed, speedKMHFlixable, (1- timeClamp));
            timeInKHM = distance / (speedKMHFlixable / 3.6f);

        }
        else 
        {
            timeInKHM = Mathf.Lerp(minHeight, maxHeight, timeClamp);
        }
        Debug.Log("timeInKHM after Lerp ? " + timeInKHM);
        ballRigidbody.ShootTo(
            currentStartPos,
            currentTargetPos,
            shotMode,
            (float)timeInKHM
        );
    }
}
