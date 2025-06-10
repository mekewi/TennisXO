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
    public Transform playerMovePosition;
    public LineRenderer lineRenderer;


    [Header("Initial Conditions")]
    public Vector3 initialVelocity;
    public Vector3 initialPosition;
    public int maxBounces = 3;

    [Header("Physics Settings")]
    public float simulationTimeStep = 0.02f;
    public float maxSimulationTime = 5f;
    public float groundHeight = 0f;

    [Header("Physics Materials")]
    public PhysicsMaterial ballMaterial;
    public PhysicsMaterial courtMaterial;
    public PlayerMovement playerMovement;

    private void Awake()
    {
        swipeInput.OnShotReady += HandleSwipe;
    }
    private void HandleSwipe(Vector3 startPos, Vector3 endPos, float swipeSpeed)
    {
        var ballRigidbody = Instantiate(ball).GetComponent<TennisBallLauncher>();
        var swipDeference = (endPos - startPos);
        ballRigidbody.transform.position = ball.transform.position;
        target.position = (new Vector3(0, 0, 0) + swipDeference.normalized) * swipDeference.magnitude;
        Vector3 currentStartPos = ballRigidbody.transform.position;
        var deference = target.position - currentStartPos;
        var dir = deference.normalized;
        var distance = deference.magnitude;
        Debug.Log("dir > " + dir);
        Debug.Log("distance > " + distance);
        Vector3 currentTargetPos = target.position;
        var timeClamp = Mathf.Clamp(swipeSpeed, 0.1f, maxTime);
        shotMode = timeClamp < 1 ? ShotControlMode.UseFlightTime : ShotControlMode.UseArcHeight;
        timeClamp = timeClamp / maxTime;
        timeClamp = Mathf.Clamp01(timeClamp);
        Debug.Log("timeClamp > " + timeClamp + " < swipeSpeed > " + swipeSpeed);
        var height = 0f;
        double timeInKHM;
        Debug.Log("IsBallWillHit Net > " + height);
        if (shotMode == ShotControlMode.UseFlightTime)
        {
            speedKMHFlixable = maxSpeed;
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
            speedKMHFlixable = Mathf.Lerp(minSpeed, speedKMHFlixable, (1 - timeClamp));
            timeInKHM = distance / (speedKMHFlixable / 3.6f);

        }
        else
        {
            timeInKHM = Mathf.Lerp(minHeight, maxHeight, timeClamp);
        }
        Vector3 initialShotVelocity = ballRigidbody.GetVelocity(
            currentStartPos,
            currentTargetPos,
            shotMode,
            (float)timeInKHM
        );

        Debug.Log("timeInKHM after Lerp ? " + timeInKHM);
        ballRigidbody.ShootTo(
            currentStartPos,
            currentTargetPos,
            shotMode,
            (float)timeInKHM
        );
        initialVelocity = initialShotVelocity;
        initialPosition = currentStartPos;
        var maxHeightBounce = ballRigidbody.DrawPreBounceTrajectoryAndTrackMaxHeight(
            startPos: currentStartPos,
            startVel: initialShotVelocity,
            startAngularVel: Vector3.zero,
            predictionTime: 5f,         // Duration of prediction
            timeStep: 0.02f,            // Step size
            lineRendererToDraw: lineRenderer
        );
        playerMovePosition.position = maxHeightBounce;
        playerMovement.StartMove(maxHeightBounce);

        //SimulateTrajectory();
    }
    public void SimulateTrajectory()
    {
        List<Vector3> preBouncePath = new List<Vector3>();

        Vector3 velocity = initialVelocity;
        Vector3 position = initialPosition;

        float combinedBounciness = Mathf.Max(ballMaterial.bounciness, courtMaterial.bounciness);
        float combinedFriction = Mathf.Sqrt(ballMaterial.dynamicFriction * courtMaterial.dynamicFriction);

        int bounces = 0;
        float time = 0f;

        bool hasBounced = false;
        Vector3 maxHeightAfterBounce = new Vector3();

        preBouncePath.Add(position);

        Vector3 postBounceVelocity = Vector3.zero;
        Vector3 postBouncePosition = Vector3.zero;

        // First loop: simulate and record only pre-bounce path
        while (time < maxSimulationTime && bounces <= maxBounces && !hasBounced)
        {
            velocity += Physics.gravity * simulationTimeStep;
            Vector3 nextPosition = position + velocity * simulationTimeStep;

            // Detect bounce
            if (position.y > groundHeight && nextPosition.y <= groundHeight)
            {
                float t = (position.y - groundHeight) / (position.y - nextPosition.y);
                Vector3 hitPoint = Vector3.Lerp(position, nextPosition, t);
                hitPoint.y = groundHeight;
                preBouncePath.Add(hitPoint);

                hasBounced = true;

                // Save state at bounce
                Vector3 normal = Vector3.up;
                Vector3 reflected = Vector3.Reflect(velocity, normal);
                reflected *= combinedBounciness;

                // Apply friction
                Vector3 tangent = new Vector3(reflected.x, 0f, reflected.z);
                tangent *= (1f - combinedFriction);
                reflected.x = tangent.x;
                reflected.z = tangent.z;

                reflected *= 0.95f;
                if (Mathf.Abs(reflected.y) < 0.5f)
                    reflected.y = 0f;

                postBounceVelocity = reflected;
                postBouncePosition = hitPoint;

                break;
            }
            else
            {
                position = nextPosition;
                preBouncePath.Add(position);
            }

            time += simulationTimeStep;
        }

        // Second loop: simulate after bounce to find max height
        if (hasBounced && postBounceVelocity.y > 0f)
        {
            Vector3 p = postBouncePosition;
            Vector3 v = postBounceVelocity;
            float postTime = 0f;

            while (postTime < maxSimulationTime && v.y > 0f)
            {
                v += Physics.gravity * simulationTimeStep;
                p += v * simulationTimeStep;

                if (p.y > maxHeightAfterBounce.y)
                    maxHeightAfterBounce = p;

                postTime += simulationTimeStep;
            }
            playerMovePosition.position = maxHeightAfterBounce;
            playerMovement.StartMove(maxHeightAfterBounce);
            Debug.Log("Max height after first bounce: " + maxHeightAfterBounce.ToString("F3") + " meters");
        }
        else if (!hasBounced)
        {
            Debug.Log("Ball did not bounce.");
        }

        // Draw only pre-bounce path
        lineRenderer.positionCount = preBouncePath.Count;
        lineRenderer.SetPositions(preBouncePath.ToArray());
    }





}
