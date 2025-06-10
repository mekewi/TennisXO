using UnityEngine;

public class PlayerSensor : MonoBehaviour
{
    public float radius;
    public LayerMask ballLayer;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

        bool isFound = Physics.SphereCast(transform.position, radius,transform.forward,out RaycastHit hitInfo,radius, ballLayer);
        if (isFound)
        {
            var ball = hitInfo.transform.GetComponent<TennisBallLauncher>();
            //ball.Pause();// = false;
        }
    }
    private void OnDrawGizmos()
    {
        Gizmos.DrawWireSphere(transform.position, radius);
    }
}
