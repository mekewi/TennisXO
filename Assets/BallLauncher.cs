using System.Collections.Generic;
using UnityEditor.Overlays;
using UnityEngine;

public class BallLauncher : MonoBehaviour
{
    public CustomTennisBallRigidbody ball;
    public SwipeInput swipeInput;
    public Transform target;
    public ShotType shotType;
    public ShotData serveData = new ShotData { power = 30f, lift = 4f, drag = 0.5f, spin = 0f };
    public Dictionary<ShotType, ShotData> shotConfigs = new Dictionary<ShotType, ShotData>()
    {
        { ShotType.Flat, new ShotData { power = 25f, lift = 6f, drag = 0.6f, spin = 0f } },
        { ShotType.Topspin, new ShotData { power = 20f, lift = 5f, drag = 0.8f, spin = 4f } },
        { ShotType.Slice, new ShotData { power = 18f, lift = 4f, drag = 0.9f, spin = -5f } },
        { ShotType.Lob, new ShotData { power = 15f, lift = 10f, drag = 0.95f, spin = 0f } },
        { ShotType.Volley, new ShotData { power = 12f, lift = 3f, drag = 0.4f, spin = 0f } }
    };
    private void Awake()
    {
        shotConfigs.Add(ShotType.Serve, serveData);
        swipeInput.OnSwipe += OnSwipe;
    }
    void Update()
    {
/*        if (Input.GetKeyDown(KeyCode.Space))
        {
            Vector3 dir = (target.position - ball.transform.position).normalized;
        }*/
    }
    public void OnSwipe(Vector2 swipe, Vector3 swipeDir, float power) 
    {
        shotType = GetShotType(swipe, true, 5);
        Debug.Log("Swipe > " + swipe.normalized);
        var newBall = Instantiate(ball);
        newBall.transform.position = ball.transform.position;
        //new Vector3(swipe.x,0, swipe.y)
        newBall.Launch(swipeDir, shotConfigs[shotType]);
    }
    ShotType GetShotType(Vector2 swipe, bool isServe, float distanceToNet)
    {
        if (isServe)
            return ShotType.Serve;

        float angle = Vector2.SignedAngle(Vector2.up, swipe);
        float magnitude = swipe.magnitude;

        if (magnitude < 100f && distanceToNet < 4f)
            return ShotType.Volley;

        if (magnitude > 200f && swipe.y > 0.9f)
            return ShotType.Lob;

        if (angle >= -30f && angle <= 30f)
            return ShotType.Flat;
        if (angle > 30f && angle < 75f)
            return ShotType.Topspin;
        if (angle < -30f && angle > -75f)
            return ShotType.Slice;

        return ShotType.Flat;
    }

}