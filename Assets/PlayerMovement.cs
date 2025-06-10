using UnityEngine;

public class PlayerMovement : MonoBehaviour
{
    public Vector3 target;
    bool startMove;
    public float speedKmh;
    public float offset;
    public void StartMove(Vector3 targetPosition) 
    {
        Debug.Log("Ball Direction > "+ (targetPosition.x - transform.position.x));
        target = targetPosition;
        if (targetPosition.x - transform.position.x < 0)
        {
            target.x += offset;
        }
        else 
        {
            target.x -= offset;
        }
        target.y = 0;
        startMove = true;

    }
    // Update is called once per frame
    void Update()
    {
        if (!startMove)
        {
            return;
        }
        float speedMps = speedKmh * 1000f / 3600f; // Convert to meters/second
        transform.position = Vector3.MoveTowards(
            transform.position,
            target  ,
            speedMps * Time.deltaTime
        );
    }
}
