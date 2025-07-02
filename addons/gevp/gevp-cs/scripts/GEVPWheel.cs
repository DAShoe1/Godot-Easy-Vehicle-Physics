// Portions are Copyright (c) 2021 Dechode
// https://github.com/Dechode/Godot-Advanced-Vehicle

// Portions are Copyright (c) 2024 Baron Wittman
// https://lupine-vidya.itch.io/gdsim/devlog/677572/series-driving-simulator-workshop-mirror
using Godot;
using Godot.Collections;

[GlobalClass]
public partial class GEVPWheel : RayCast3D
{
    // The [Node3D] correlating to this Wheel, which will have its
    // rotation manipulated to make it spin and rotate.
    // [br][br]
    // [b]Tip:[/b] Make sure that your wheel is facing in the [b]+Z[/b] axis
    // as this is considered the forward direction by both Godot and this script.
    // [br][br]
    // [b]Tip:[/b] If you're having issues with positioning your wheel,
    // try parenting it to a [Node3D] and using that as the wheel node instead.
    [Export] public Node3D WheelNode;
    [ExportGroup("Internal (Don't edit!)")]
    // Internal fields which should not be set in inspector, but need to be exported to be accessible from GDScript.
    [Export] public float WheelMass = 15.0f;
    [Export] public float TireRadius = 0.3f;
    [Export] public float TireWidth = 205.0f;
    [Export] public float Ackermann = 0.15f;
    [Export] public float ContactPatch = 0.2f;
    [Export] public float BrakingGripMultiplier = 1.4f;
    [Export] public string SurfaceType = "";
    [Export] public Dictionary<string, float> TireStiffnesses = new Dictionary<string, float>
    {
        { "Road", 5.0f },
        { "Dirt", 0.5f },
        { "Grass", 0.5f }
    };
    [Export] public Dictionary<string, float> CoefficientOfFriction = new Dictionary<string, float>
    {
        { "Road", 2.0f },
        { "Dirt", 1.4f },
        { "Grass", 1.0f }
    };
    [Export] public Dictionary<string, float> RollingResistance = new Dictionary<string, float>
    {
        { "Road", 1.0f },
        { "Dirt", 2.0f },
        { "Grass", 4.0f }
    };
    [Export] public Dictionary<string, float> LateralGripAssist = new Dictionary<string, float>
    {
        { "Road", 0.05f },
        { "Dirt", 0.0f },
        { "Grass", 0.0f }
    };
    [Export] public Dictionary<string, float> LongitudinalGripRatio = new Dictionary<string, float>
    {
        { "Road", 0.5f },
        { "Dirt", 0.5f },
        { "Grass", 0.5f }
    };

    [Export] public float SpringLength = 0.15f;
    [Export] public float SpringRate = 0.0f;
    [Export] public float SlowBump = 0.0f;
    [Export] public float FastBump = 0.0f;
    [Export] public float SlowRebound = 0.0f;
    [Export] public float FastRebound = 0.0f;
    [Export] public float FastDampThreshold = 127.0f;
    [Export] public float Antiroll = 0.0f;
    [Export] public float Toe = 0.0f;
    [Export] public float BumpStopMultiplier = 1.0f;
    [Export] public float WheelToBodyTorqueMultiplier = 0.0f;
    [Export] public float MassOverWheel = 0.0f;

    [Export] public float WheelMoment = 0.0f;
    [Export] public float Spin = 0.0f;
    [Export] public float SpinVelocityDiff = 0.0f;
    [Export] public float SpringForce = 0.0f;
    [Export] public float AppliedTorque = 0.0f;
    [Export] public Vector3 LocalVelocity = Vector3.Zero;
    [Export] public Vector3 PreviousVelocity = Vector3.Zero;
    [Export] public Vector3 PreviousGlobalPosition = Vector3.Zero;
    [Export] public Vector2 ForceVector = Vector2.Zero;
    [Export] public Vector2 SlipVector = Vector2.Zero;
    [Export] public float PreviousCompression = 0.0f;
    [Export] public float SpringCurrentLength = 0.0f;
    [Export] public float MaxSpringLength = 0.0f;
    [Export] public float AntirollForce = 0.0f;
    [Export] public float DampingForce = 0.0f;
    [Export] public float SteeringRatio = 0.0f;
    [Export] public Node LastCollider;
    [Export] public Vector3 LastCollisionPoint = Vector3.Zero;
    [Export] public Vector3 LastCollisionNormal = Vector3.Zero;
    [Export] public float CurrentCof = 0.0f;
    [Export] public float CurrentRollingResistance = 0.0f;
    [Export] public float CurrentLateralGripAssist = 0.0f;
    [Export] public float CurrentLongitudinalGripRatio = 0.0f;
    [Export] public float CurrentTireStiffness = 0.0f;
    [Export] public float AbsEnableTime = 0.0f;
    [Export] public float AbsPulseTime = 0.3f;
    [Export] public float AbsSpinDifferenceThreshold = -12.0f;
    [Export] public bool LimitSpin = false;
    [Export] public bool IsDriven = false;
    [Export] public GEVPWheel OppositeWheel;
    [Export] public float BeamAxle = 0.0f;

    [Export] public GEVPVehicle Vehicle;


    public override void _Process(double deltaD)
    {
        var delta = (float)deltaD;
        if (WheelNode != null)
        {
            WheelNode.Position = new Vector3(WheelNode.Position.X, Mathf.Min(0.0f, -SpringCurrentLength), WheelNode.Position.Z);

            if (!Mathf.IsZeroApprox(BeamAxle))
            {
                Vector3 wheelLookatVector = (OppositeWheel.Transform * OppositeWheel.WheelNode.Position) - (Transform * WheelNode.Position);
                WheelNode.Rotation = new Vector3(WheelNode.Rotation.X, WheelNode.Rotation.Y, wheelLookatVector.AngleTo(Vector3.Right * BeamAxle) * Mathf.Sign(wheelLookatVector.Y * BeamAxle));
            }

            WheelNode.Rotation = new Vector3(WheelNode.Rotation.X - Mathf.Wrap(Spin * delta, 0, Mathf.Tau), WheelNode.Rotation.Y, WheelNode.Rotation.Z);
        }
    }
    public void Initialize()
    {
        WheelNode.RotationOrder = EulerOrder.Zxy;
        WheelMoment = 0.5f * WheelMass * Mathf.Pow(TireRadius, 2);
        SetTargetPosition(Vector3.Down * (SpringLength + TireRadius));
        Vehicle = GetParent<GEVPVehicle>();
        MaxSpringLength = SpringLength;
        CurrentCof = CoefficientOfFriction[SurfaceType];
        CurrentRollingResistance = RollingResistance[SurfaceType];
        CurrentLateralGripAssist = LateralGripAssist[SurfaceType];
        CurrentLongitudinalGripRatio = LongitudinalGripRatio[SurfaceType];
        CurrentTireStiffness = 1000000.0f + 8000000.0f * TireStiffnesses[SurfaceType];
    }

    public void Steer(float input, float maxSteeringAngle)
    {
        input *= SteeringRatio;
        var rotation = Rotation;
        rotation.Y = (maxSteeringAngle * (input + (1 - Mathf.Cos(input * 0.5f * Mathf.Pi)) * Ackermann)) + Toe;
        Rotation = rotation;
    }

    public float ProcessTorque(float drive, float driveInertia, float brakeTorque, bool allowAbs, float delta)
    {
        // Add the torque the wheel produced last frame from surface friction
        float netTorque = ForceVector.Y * TireRadius;
        float previousSpin = Spin;
        netTorque += drive;

        // If antilock brakes are still active, don't apply brake torque
        if (AbsEnableTime > Vehicle.DeltaTime)
        {
            brakeTorque = 0.0f;
            allowAbs = false;
        }

        // If the wheel slip from braking is too great, enable the antilock brakes
        if (Mathf.Abs(Spin) > 5.0f && SpinVelocityDiff < AbsSpinDifferenceThreshold)
        {
            if (allowAbs && brakeTorque > 0.0f)
            {
                brakeTorque = 0.0f;
                AbsEnableTime = Vehicle.DeltaTime + AbsPulseTime;
            }
        }

        // Applied torque is used to ensure the wheels don't apply more force
        // than the motor or brakes applied to the wheel
        float appliedTorque;
        if (Mathf.IsZeroApprox(Spin))
        {
            appliedTorque = Mathf.Abs(drive - brakeTorque);
        }
        else
        {
            appliedTorque = Mathf.Abs(drive - (brakeTorque * Mathf.Sign(Spin)));
        }

        // If braking and nearly stopped, just stop the wheel completely.
        if (Mathf.Abs(Spin) < 5.0f && brakeTorque > Mathf.Abs(netTorque))
        {
            if (allowAbs && Mathf.Abs(LocalVelocity.Z) > 2.0f)
            {
                AbsEnableTime = Vehicle.DeltaTime + AbsPulseTime;
            }
            else
            {
                Spin = 0.0f;
            }
        }
        else
        {
            // Spin the wheel based on the provided torque. The tire forces will handle
            // applying that force to the vehicle.
            netTorque -= brakeTorque * Mathf.Sign(Spin);
            float newSpin = Spin + ((netTorque / (WheelMoment + driveInertia)) * delta);
            if (Mathf.Sign(Spin) != Mathf.Sign(newSpin) && brakeTorque > Mathf.Abs(drive))
            {
                newSpin = 0.0f;
            }
            Spin = newSpin;
        }

        // The returned value is used to track wheel speed difference
        if (Mathf.IsZeroApprox(drive * delta))
        {
            return 0.5f;
        }
        else
        {
            return (Spin - previousSpin) * (WheelMoment + driveInertia) / (drive * delta);
        }
    }

    public float ProcessForces(float oppositeCompression, bool braking, float delta)
    {
        ForceRaycastUpdate();
        PreviousVelocity = LocalVelocity;
        LocalVelocity = (GlobalPosition - PreviousGlobalPosition) / delta * GlobalTransform.Basis;
        PreviousGlobalPosition = GlobalPosition;

        // Determine the surface the tire is on. Uses node groups
        if (IsColliding())
        {
            LastCollider = (Node)GetCollider();
            LastCollisionPoint = GetCollisionPoint();
            LastCollisionNormal = GetCollisionNormal();
            var surfaceGroups = LastCollider.GetGroups();
            if (surfaceGroups.Count > 0)
            {
                if (SurfaceType != surfaceGroups[0])
                {
                    SurfaceType = surfaceGroups[0];
                    CurrentCof = CoefficientOfFriction[SurfaceType];
                    CurrentRollingResistance = RollingResistance[SurfaceType];
                    CurrentLateralGripAssist = LateralGripAssist[SurfaceType];
                    CurrentLongitudinalGripRatio = LongitudinalGripRatio[SurfaceType];
                    CurrentTireStiffness = 1000000.0f + 8000000.0f * TireStiffnesses[SurfaceType];
                }
            }
        }
        else
        {
            LastCollider = null;
        }

        float compression = ProcessSuspension(oppositeCompression, delta);

        if (IsColliding() && LastCollider != null)
        {
            ProcessTires(braking, delta);
            Vector3 contact = LastCollisionPoint - Vehicle.GlobalPosition;
            if (SpringForce > 0.0f)
            {
                Vehicle.ApplyForce(LastCollisionNormal * SpringForce, contact);
            }
            else
            {
                // Apply a small amount of downward force if there is no spring force
                Vehicle.ApplyForce(-GlobalTransform.Basis.Y * Vehicle.Mass, GlobalPosition - Vehicle.GlobalPosition);
            }

            Vehicle.ApplyForce(GlobalTransform.Basis.X * ForceVector.X, contact);
            Vehicle.ApplyForce(GlobalTransform.Basis.Z * ForceVector.Y, contact);

            // Applies a torque on the vehicle body centered on the wheel. Gives the vehicle
            // more weight transfer when the center of gravity is really low.
            if (braking)
            {
                WheelToBodyTorqueMultiplier = 1.0f / (BrakingGripMultiplier + 1.0f);
            }
            Vehicle.ApplyForce(-GlobalTransform.Basis.Y * ForceVector.Y * 0.5f * WheelToBodyTorqueMultiplier, ToGlobal(Vector3.Forward * TireRadius));
            Vehicle.ApplyForce(GlobalTransform.Basis.Y * ForceVector.Y * 0.5f * WheelToBodyTorqueMultiplier, ToGlobal(Vector3.Back * TireRadius));

            return compression;
        }
        else
        {
            ForceVector = Vector2.Zero;
            SlipVector = Vector2.Zero;
            Spin -= Mathf.Sign(Spin) * delta * 2.0f / WheelMoment;
            return 0.0f;
        }
    }

    public float ProcessSuspension(float oppositeCompression, float delta)
    {
        if (IsColliding() && LastCollider != null)
        {
            SpringCurrentLength = LastCollisionPoint.DistanceTo(GlobalPosition) - TireRadius;
        }
        else
        {
            SpringCurrentLength = SpringLength;
        }

        bool noContact = false;
        if (SpringCurrentLength > MaxSpringLength)
        {
            SpringCurrentLength = MaxSpringLength;
            noContact = true;
        }

        bool bottomOut = false;
        if (SpringCurrentLength < 0.0f)
        {
            SpringCurrentLength = 0.0f;
            bottomOut = true;
        }

        float compression = (SpringLength - SpringCurrentLength) * 1000.0f;

        float springSpeedMmPerSeconds = (compression - PreviousCompression) / delta;
        PreviousCompression = compression;

        SpringForce = compression * SpringRate;
        AntirollForce = Antiroll * (compression - oppositeCompression);
        SpringForce += AntirollForce;

        // If the suspension is bottomed out, apply additional forces to prevent vehicle body collisions.
        float bottomOutDamping = 0.0f;
        float bottomOutDampingFast = 0.0f;
        float bottomOutForce = 0.0f;
        if (bottomOut)
        {
            float gravityOnSpring = Mathf.Clamp(GlobalTransform.Basis.Y.Dot(-Vehicle.CurrentGravity.Normalized()), 0.0f, 1.0f);
            bottomOutForce = (((MassOverWheel * Mathf.Clamp(springSpeedMmPerSeconds * 0.001f, 0.0f, 5.0f)) / delta) + (MassOverWheel * Vehicle.CurrentGravity.Length() * gravityOnSpring)) * BumpStopMultiplier;
            bottomOutDamping = -SlowBump;
            bottomOutDampingFast = -FastBump;
        }

        if (springSpeedMmPerSeconds >= 0)
        {
            if (springSpeedMmPerSeconds > FastDampThreshold)
            {
                DampingForce = ((springSpeedMmPerSeconds - FastDampThreshold) * (FastBump + bottomOutDampingFast)) + (FastDampThreshold * (SlowBump + bottomOutDamping));
            }
            else
            {
                DampingForce = springSpeedMmPerSeconds * (SlowBump + bottomOutDamping);
            }
        }
        else
        {
            if (springSpeedMmPerSeconds < -FastDampThreshold)
            {
                DampingForce = ((springSpeedMmPerSeconds + FastDampThreshold) * FastRebound) + (-FastDampThreshold * SlowRebound);
            }
            else
            {
                DampingForce = springSpeedMmPerSeconds * SlowRebound;
            }
        }

        SpringForce += DampingForce;

        SpringForce = Mathf.Max(0, SpringForce + bottomOutForce);

        MaxSpringLength = Mathf.Clamp((((SpringForce / WheelMass) - springSpeedMmPerSeconds) * delta * 0.001f) + SpringCurrentLength, 0.0f, SpringLength);

        if (noContact)
        {
            SpringForce = 0.0f;
        }

        return compression;
    }

    public void ProcessTires(bool braking, float delta)
    {
        // This is a modified version of the brush tire model that removes the friction falloff beyond
        // the peak grip level.
        Vector2 localPlanar = new Vector2(LocalVelocity.X, LocalVelocity.Z).Normalized() * Mathf.Clamp(LocalVelocity.Length(), 0.0f, 1.0f);
        SlipVector.X = Mathf.Asin(Mathf.Clamp(-localPlanar.X, -1.0f, 1.0f));
        SlipVector.Y = 0.0f;

        float wheelVelocity = Spin * TireRadius;
        SpinVelocityDiff = wheelVelocity + LocalVelocity.Z;
        float neededRollingForce = ((SpinVelocityDiff * WheelMoment) / TireRadius) / delta;
        float maxYForce = 0.0f;

        // Because the amount of force the tire applies is based on the amount of slip,
        // a maximum force is calculated based on the applied engine torque to prevent
        // the tire from creating too much force.
        if (Mathf.Abs(AppliedTorque) > Mathf.Abs(neededRollingForce))
        {
            maxYForce = Mathf.Abs(AppliedTorque / TireRadius);
        }
        else
        {
            maxYForce = Mathf.Abs(neededRollingForce / TireRadius);
        }

        float maxXForce = Mathf.Abs(MassOverWheel * LocalVelocity.X) / delta;

        float zSign = Mathf.Sign(-LocalVelocity.Z);
        if (LocalVelocity.Z == 0.0f)
        {
            zSign = 1.0f;
        }

        SlipVector.Y = ((Mathf.Abs(LocalVelocity.Z) - (wheelVelocity * zSign)) / (1.0f + Mathf.Abs(LocalVelocity.Z)));

        if (SlipVector.IsZeroApprox())
        {
            SlipVector = new Vector2(0.0001f, 0.0001f);
        }

        float corneringStiffness = 0.5f * CurrentTireStiffness * Mathf.Pow(ContactPatch, 2.0f);
        float friction = CurrentCof * SpringForce - (SpringForce / (TireWidth * ContactPatch * 0.2f));
        float deflect = 1.0f / (Mathf.Sqrt(Mathf.Pow(corneringStiffness * SlipVector.Y, 2.0f) + Mathf.Pow(corneringStiffness * SlipVector.X, 2.0f)));

        // Adds in additional longitudinal grip when braking
        float brakingHelp = 1.0f;
        if (SlipVector.Y > 0.3f && braking)
        {
            brakingHelp = (1 + (BrakingGripMultiplier * Mathf.Clamp(Mathf.Abs(SlipVector.Y), 0.0f, 1.0f)));
        }

        float critLength = friction * (1.0f - SlipVector.Y) * ContactPatch * (0.5f * deflect);
        if (critLength >= ContactPatch)
        {
            ForceVector.Y = corneringStiffness * SlipVector.Y / (1.0f - SlipVector.Y);
            ForceVector.X = corneringStiffness * SlipVector.X / (1.0f - SlipVector.Y);
        }
        else
        {
            float brushX = (1.0f - friction * (1.0f - SlipVector.Y) * (0.25f * deflect)) * deflect;
            ForceVector.Y = friction * CurrentLongitudinalGripRatio * corneringStiffness * SlipVector.Y * brushX * brakingHelp * zSign;
            ForceVector.X = friction * corneringStiffness * SlipVector.X * brushX * (Mathf.Abs(SlipVector.X * CurrentLateralGripAssist) + 1.0f);
        }

        if (Mathf.Abs(ForceVector.Y) > Mathf.Abs(maxYForce))
        {
            ForceVector.Y = maxYForce * Mathf.Sign(ForceVector.Y);
            LimitSpin = true;
        }
        else
        {
            LimitSpin = false;
        }

        if (Mathf.Abs(ForceVector.X) > maxXForce)
        {
            ForceVector.X = maxXForce * Mathf.Sign(ForceVector.X);
        }

        ForceVector.Y -= ProcessRollingResistance() * Mathf.Sign(LocalVelocity.Z);
    }

    public float ProcessRollingResistance()
    {
        float rollingResistanceCoefficient = 0.005f + (0.5f * (0.01f + (0.0095f * Mathf.Pow(LocalVelocity.Z * 0.036f, 2))));
        return rollingResistanceCoefficient * SpringForce * CurrentRollingResistance;
    }
    public float GetReactionTorque()
    {
        return ForceVector.Y * TireRadius;
    }

    public float GetFriction(float normalForce, string surface)
    {
        float surfaceCof = 1.0f;
        if (CoefficientOfFriction.ContainsKey(surface))
        {
            surfaceCof = CoefficientOfFriction[surface];
        }
        return surfaceCof * normalForce - (normalForce / (TireWidth * ContactPatch * 0.2f));
    }
}
