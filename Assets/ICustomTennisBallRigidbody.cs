using System;
using UnityEngine;

public interface ICustomTennisBallRigidbody
{
    event Action OnBounce;
    event Action OnStop;

    void AddForce(Vector3 force);
    Vector3 GetVelocity();
    Vector3 GetVelocity(Vector3 startPos, Vector3 targetPos, ShotControlMode mode, float value);
    float IsBallWillHit(Vector3 startPos, Vector3 targetPos, float netHeight, float value);
    bool IsMoving();
    void ResetBall(Vector3 position);
    void SetVelocity(Vector3 initialVelocity);
    void ShootTo(Vector3 startPos, Vector3 targetPos, ShotControlMode mode, float value, Vector3 initialSpinAngularVelocity = default(Vector3));
}