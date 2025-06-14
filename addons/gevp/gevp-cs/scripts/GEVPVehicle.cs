// Portions are Copyright (c) 2021 Dechode
// https://github.com/Dechode/Godot-Advanced-Vehicle
using System.Linq;
using Godot;
using Godot.Collections;
[GlobalClass]
public partial class GEVPVehicle : RigidBody3D
{
    [ExportGroup("Wheel Nodes")]
    // Assign this to the Wheel [RayCast3D] that is this vehicle's front left wheel.
    [Export] public GEVPWheel FrontLeftWheel;
    // Assign this to the Wheel [RayCast3D] that is this vehicle's front right wheel.
    [Export] public GEVPWheel FrontRightWheel;
    // Assign this to the Wheel [RayCast3D] that is this vehicle's rear left wheel.
    [Export] public GEVPWheel RearLeftWheel;
    // Assign this to the Wheel [RayCast3D] that is this vehicle's rear right wheel.
    [Export] public GEVPWheel RearRightWheel;

    [ExportGroup("Steering")]
    // The rate that the steering input changes in order to smooth
    // out direction changes to the wheel.
    // Steering input is between -1 and 1. Speed is in units per second.
    [Export] public float SteeringSpeed = 4.25f;
    // The rate that the steering input changes when steering back to center.
    // Speed is in units per second.
    [Export] public float CountersteerSpeed = 11.0f;
    // Reduces steering input based on the vehicle's speed.
    // Steering speed is divided by the velocity at this magnitude.
    // The larger the number, the slower the steering at speed.
    [Export] public float SteeringSpeedDecay = 0.20f;
    // Further steering input is prevented if the wheels' lateral slip is greater than this number.
    [Export] public float SteeringSlipAssist = 0.15f;
    // The magnitude to adjust steering toward the direction of travel based on the vehicle's lateral velocity.
    [Export] public float CountersteerAssist = 0.9f;
    // Steering input is raised to the power of this number.
    // This has the effect of slowing steering input near the limits.
    [Export] public float SteeringExponent = 1.5f;
    // The maximum steering angle in radians.
    // [br][br]
    // [b]Note:[/b] This property is edited in the inspector in degrees. If you want to use degrees in a script, use [code]deg_to_rad[/code].
    [Export(PropertyHint.Range, "0,360,0.1,radians_as_degrees")] public float MaxSteeringAngle = 0.7f;

    [ExportSubgroup("Front Axle", "Front")]
    // The ratio that the wheels turn based on steering input.
    // [br]The higher this value, the more the wheels will turn due to steering input.
    [Export] public float FrontSteeringRatio = 1.0f;
    // Ackermann wheel steering angle correction
    // [Export] public float FrontAckermann = 0.15f;
    [ExportSubgroup("Rear Axle", "Rear")]
    // The ratio the wheels turn based on steering input.
    // [br]The higher this value, the more the wheels will turn due to steering input.
    [Export] public float RearSteeringRatio = 0.0f;


    [ExportGroup("Throttle and Braking")]
    // The rate the throttle input changes to smooth input.
    // Throttle input is between 0 and 1. Speed is in units per second.
    [Export] public float ThrottleSpeed = 20.0f;
    // Multiply the throttle speed by this based on steering input.
    [Export] public float ThrottleSteeringAdjust = 0.1f;
    // The rate braking input changes to smooth input.
    // Braking input is between 0 and 1. Speed is in units per second.
    [Export] public float BrakingSpeed = 10.0f;
    // Multiplies the automatically calculated brake force.
    [Export] public float BrakeForceMultiplier = 1.0f;
    // Ratio of total brake force applied as front wheels : back wheels. If this value is
    // below 0.0, this value will be automatically calculated instead.
    [Export] public float FrontBrakeBias = -1.0f;
    // Prevents engine power from causing the tires to slip beyond this value.
    // Values below 0 disable the effect.
    [Export] public float TractionControlMaxSlip = 8.0f;

    [ExportSubgroup("Front Axle", "Front")]
    // How long the ABS releases the brake, in seconds, when the
    // spin threshold is crossed.
    [Export] public float FrontAbsPulseTime = 0.03f;
    // The difference in speed required between the wheel and the
    // driving surface for ABS to engage.
    [Export] public float FrontAbsSpinDifferenceThreshold = 12.0f;

    [ExportSubgroup("Rear Axle", "Rear")]
    // How long the ABS releases the brake, in seconds, when the
    // spin threshold is crossed.
    [Export] public float RearAbsPulseTime = 0.03f;
    // The difference in speed required between the wheel and the
    // driving surface for ABS to engage.
    [Export] public float RearAbsSpinDifferenceThreshold = 12.0f;

    [ExportGroup("Stability")]
    // Stablity applies torque forces to the vehicle body when the body angle
    // relative to the direction of travel exceeds a threshold.
    [Export] public bool EnableStability = true;
    // The yaw angle the vehicle must reach before stability is applied.
    // Based on the dot product, 0 being straight, 1 being 90 degrees
    [Export] public float StabilityYawEngageAngle = 0.0f;
    // Strength multiplier for the applied yaw correction.
    [Export] public float StabilityYawStrength = 6.0f;
    // Additional strength multiplier for a grounded vehicle to overcome traction.
    [Export] public float StabilityYawGroundMultiplier = 2.0f;
    // A multiplier for the torque used to keep the vehicle upright while airbourn.
    [Export] public float StabilityUprightSpring = 1.0f;
    // A multiplier for the torque used to dampen rotation while airborne.
    [Export] public float StabilityUprightDamping = 1000.0f;

    [ExportGroup("Motor")]
    // Maximum motor torque in NM.
    [Export] public float MaxTorque = 300.0f;
    // Maximum motor RPM.
    [Export] public float MaxRpm = 7000.0f;
    // Idle motor RPM.
    [Export] public float IdleRpm = 1000.0f;
    // Percentage of torque produced across the RPM range.
    [Export] public Curve TorqueCurve;
    // Variable motor drag based on RPM.
    [Export] public float MotorDrag = 0.005f;
    // Constant motor drag.
    [Export] public float MotorBrake = 10.0f;
    // Moment of inertia for the motor.
    [Export] public float MotorMoment = 0.5f;
    // The motor will use this rpm when launching from a stop.
    [Export] public float ClutchOutRpm = 3000.0f;
    // Max clutch torque as a ratio of max motor torque.
    [Export] public float MaxClutchTorqueRatio = 1.6f;


    [ExportGroup("Gearbox")]
    // Transmission gear ratios, the size of the array determines the number of gears
    [Export] public Array<float> GearRatios = [3.8f, 2.3f, 1.7f, 1.3f, 1.0f, 0.8f];
    // Final drive ratio
    [Export] public float FinalDrive = 3.2f;
    // Reverse gear ratio
    [Export] public float ReverseRatio = 3.3f;
    // Time it takes to change gears on up shifts in seconds
    [Export] public float ShiftTime = 0.3f;
    // Enables automatic gear changes
    [Export] public bool AutomaticTransmission = true;
    // Timer to prevent the automatic gear shifts changing gears too quickly 
    // in milliseconds
    [Export] public float AutomaticTimeBetweenShifts = 1000.0f;
    // Drivetrain inertia
    [Export] public float GearInertia = 0.02f;


    [ExportGroup("Drivetrain")]
    // Torque delivered to the front wheels vs the rear wheels.
    // Value of 1 is FWD, a value of 0 is RWD, anything in between is AWD.
    [Export] public float FrontTorqueSplit = 0.0f;
    // When enabled, the torque split will change based on wheel slip.
    [Export] public bool VariableTorqueSplit = false;
    // Torque split to interpolate toward when there is wheel slip. Variable Torque
    // Split must be enabled.
    [Export] public float FrontVariableSplit = 0.0f;
    // How quickly to interpolate between torque splits in seconds.
    [Export] public float VariableSplitSpeed = 1.0f;
    [ExportSubgroup("Front Axle", "Front")]
    // The wheels of the axle will be forced to spin the same speed if there
    // is at least this much torque applied. Keeps vehicle from spinning one wheel.
    // Torque is measured after multiplied by the current gear ratio.
    // Negative values will disable.
    [Export] public float FrontLockingDifferentialEngageTorque = 200.0f;
    // The amount of torque vectoring to apply to the axle based on steering input.
    // Only functions if the differential is locked.
    // A value of 1.0 would apply all torque to the outside wheel.
    [Export] public float FrontTorqueVectoring = 0.0f;
    [ExportSubgroup("Rear Axle", "Rear")]
    // The wheels of the axle will be forced to spin the same speed if there
    // is at least this much torque applied. Keeps vehicle from spinning one wheel.
    // Torque is measured after multiplied by the current gear ratio.
    // Negative values will disable.
    [Export] public float RearLockingDifferentialEngageTorque = 200.0f;
    // The amount of torque vectoring to apply to the axle based on steering input.
    // Only functions if the differential is locked.
    // A value of 1.0 would apply all torque to the outside wheel.
    [Export] public float RearTorqueVectoring = 0.0f;

    [ExportGroup("Suspension")]
    // Vehicle mass in kilograms.
    [Export] public float VehicleMass = 1500.0f;
    // The percentage of the vehicle mass over the front axle.
    [Export] public float FrontWeightDistribution = 0.5f;
    // The center of gravity is calculated from the front weight distribution
    // with the height centered on the wheel raycast positions. This will offset
    // the height from that calculated position.
    [Export] public float CenterOfGravityHeightOffset = -0.2f;
    // Multiplies the calculated inertia by this value.
    // Greater inertia values will cause more force to be
    // required to rotate the car.
    [Export] public float InertiaMultiplier = 1.2f;

    [ExportSubgroup("Front Axle", "Front")]
    // The amount of suspension travel in meters.
    [Export] public float FrontSpringLength = 0.15f;
    // How much the spring is compressed when the vehicle is at rest.
    // This is used to calculate the approriate spring rate for the wheel.
    // A value of 0 would be a fully compressed spring.
    [Export] public float FrontRestingRatio = 0.5f;
    // Damping ratio is used to calculate the damping forces on the spring.
    // A value of 1 would be critically damped. Passenger cars typically have a
    // ratio around 0.3, while a race car could be as high as 0.9.
    [Export] public float FrontDampingRatio = 0.4f;
    // Bump damping multiplier applied to the damping force calulated from the
    // damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
    // 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
    [Export] public float FrontBumpDampMultiplier = 0.6667f;
    // Rebound damping multiplier applied to the damping force calulated from the
    // damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
    // 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
    [Export] public float FrontReboundDampMultiplier = 1.5f;
    // Antiroll bar stiffness as a ratio to spring stiffness.
    [Export] public float FrontArbRatio = 0.25f;
    // Wheel camber isn't simulated, but giving the raycast a slight angle helps
    // with simulation stability. Measured in radians.
    [Export] public float FrontCamber = 0.01745329f;
    // Toe of the tires measured in radians.
    [Export] public float FrontToe = 0.01f;
    // Multiplier for the force applied when the suspension is fully compressed.
    // If the vehicle bounces off large bumps, reducing this will help.
    [Export] public float FrontBumpStopMultiplier = 1.0f;
    // If true the wheels of this axle will be aligned as if they were attached to
    // a beam axle. This setting does not affect vehicle handling.
    [Export] public bool FrontBeamAxle = false;

    [ExportSubgroup("Rear Axle", "Rear")]
    // The amount of suspension travel in meters. Rear suspension typically has
    // more travel than the front.
    [Export] public float RearSpringLength = 0.2f;
    // How much the spring is compressed when the vehicle is at rest.
    // This is used to calculate the approriate spring rate for the wheel.
    // A value of 1 would be a fully compressed spring. With a value of 0.5 the
    // suspension will rest at the center of it's length.
    [Export] public float RearRestingRatio = 0.5f;
    // Damping ratio is used to calculate the damping forces on the spring.
    // A value of 1 would be critically damped. Passenger cars typically have a
    // ratio around 0.3, while a race car could be as high as 0.9.
    [Export] public float RearDampingRatio = 0.4f;
    // Bump damping multiplier applied to the damping force calulated from the
    // damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
    // 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
    [Export] public float RearBumpDampMultiplier = 0.6667f;
    // Rebound damping multiplier applied to the damping force calulated from the
    // damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
    // 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
    [Export] public float RearReboundDampMultiplier = 1.5f;
    // Antiroll bar stiffness as a ratio to spring stiffness.
    [Export] public float RearArbRatio = 0.25f;
    // Wheel camber isn't simulated, but giving the raycast a slight angle helps
    // with simulation stability.
    [Export] public float RearCamber = 0.01745329f;
    // Toe of the tires measured in radians.
    [Export] public float RearToe = 0.01f;
    // Multiplier for the force applied when the suspension is fully compressed.
    // If the vehicle bounces off large bumps, reducing this will help.
    [Export] public float RearBumpStopMultiplier = 1.0f;
    // If true the wheels of this axle will be aligned as if they were attached to
    // a beam axle. This setting does not affect vehicle handling.
    [Export] public bool RearBeamAxle = false;

    [ExportGroup("Tires")]
    // Represents the length of the tire contact patch in the brush tire model.
    [Export] public float ContactPatch = 0.2f;
    // Provides additional longitudinal grip when braking.
    [Export] public float BrakingGripMultiplier = 1.5f;
    // Tire force applied to the ground is also applied to the vehicle body as a
    // torque centered on the wheel. 
    [Export] public float WheelToBodyTorqueMultiplier = 1.0f;
    // Represents tire stiffness in the brush tire model. Higher values increase
    // the responsivness of the tire.
    // Surface detection uses node groups to identify the surface, so make sure
    // your staticbodies and rigidbodies belong to one of these groups.
    [Export]
    public Dictionary<string, float> TireStiffnesses = new Dictionary<string, float>
    {
        { "Road", 10.0f },
        { "Dirt", 0.5f },
        { "Grass", 0.5f }
    };
    // A multiplier for the amount of force a tire can apply based on the surface.
    // Surface detection uses node groups to identify the surface, so make sure
    // your staticbodies and rigidbodies belong to one of these groups.
    [Export]
    public Dictionary<string, float> CoefficientOfFriction = new Dictionary<string, float>
    {
        { "Road", 3.0f },
        { "Dirt", 2.4f },
        { "Grass", 2.0f }
    };
    // A multiplier for the amount of rolling resistance force based on the surface.
    // Surface detection uses node groups to identify the surface, so make sure
    // your staticbodies and rigidbodies belong to one of these groups.
    [Export]
    public Dictionary<string, float> RollingResistance = new Dictionary<string, float>
    {
        { "Road", 1.0f },
        { "Dirt", 2.0f },
        { "Grass", 4.0f }
    };
    // A multiplier to provide more grip based on the amount of lateral wheel slip.
    // This can be used to keep vehicles from sliding a long distance, but may provide
    // unrealistically high amounts of grip.
    // Surface detection uses node groups to identify the surface, so make sure
    // your staticbodies and rigidbodies belong to one of these groups.
    [Export]
    public Dictionary<string, float> LateralGripAssist = new Dictionary<string, float>
    {
        { "Road", 0.05f },
        { "Dirt", 0.0f },
        { "Grass", 0.0f }
    };

    // A multiplier to adjust longitudinal grip to differ from lateral grip.
    // Useful for allowing vehicles to have wheel spin and maintain high lateral grip.
    // Surface detection uses node groups to identify the surface, so make sure
    // your staticbodies and rigidbodies belong to one of these groups.
    [Export]
    public Dictionary<string, float> LongitudinalGripRatio = new Dictionary<string, float>
    {
        { "Road", 0.5f },
        { "Dirt", 0.5f },
        { "Grass", 0.5f }
    };
    [ExportSubgroup("Front Axle", "Front")]
    // Tire radius in meters
    [Export] public float FrontTireRadius = 0.3f;
    // Tire width in millimeters. The width doesn't directly affect tire friction,
    // but reduces the effects of tire load sensitivity.
    [Export] public float FrontTireWidth = 245.0f;
    // Wheel mass in kilograms.
    [Export] public float FrontWheelMass = 15.0f;
    [ExportSubgroup("Rear Axle", "Rear")]
    // Tire radius in meters
    [Export] public float RearTireRadius = 0.3f;
    // Tire width in millimeters. The width doesn't directly affect tire friction,
    // but reduces the effects of tire load sensitivity.
    [Export] public float RearTireWidth = 245.0f;
    // Wheel mass in kilograms.
    [Export] public float RearWheelMass = 15.0f;


    [ExportGroup("Aerodynamics")]
    // The drag coefficient quantifies how much [b]drag[/b] (force against thrust)
    // the vehicle recieves when moving through air. In the drag equation,
    // a lower drag coefficient means the vehicle will experience less drag
    // force, allowing it to move faster.
    // [br]Typically, the drag coefficient is assumed from the shape of the
    // body, where more teardrop-shaped bodies experience a lower drag coefficient.
    // Un-streamlined cyllindrical bodies have a drag coefficient of
    // around [code]0.80[/code], while more streamlined teardrop-shaped bodies 
    // can have a drag coefficient as low as [code]0.05[/code], or even lower.
    // [br]As a more relavant example, most cars have drag coefficients
    // around [code]0.40[/code].
    [Export] public float CoefficientOfDrag = 0.3f;
    // From [url=https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/density.html#:~:text=Halving%20the%20density%20halves%20the,above%20which%20it%20cannot%20fly.]NASA[/url]:
    // [i]"Halving the density halves the lift, halving the density halves the drag. The [lb]air[rb] density depends on the type of [lb]air[rb] and the depth of the [lb]air[rb]. In the atmosphere, air density decreases as altitude increases. This explains why airplanes have a flight ceiling, an altitude above which it cannot fly."[/i]
    [Export] public float AirDensity = 1.225f;
    // The amount of surface area the front-facing part of the vehicle has,
    // in meters squared ([code]m^2[/code]).
    // [br][br]
    // [b]Note:[/b] You do not have to calculate this value to be exact,
    // a rough estimate - or even something completely different, depending
    // on the result you want - will do.
    [Export] public float FrontalArea = 2.0f;

    public const float AngularVelocityToRpm = 60.0f / Mathf.Tau;




    [ExportGroup("Inputs")]
    // C# fields must be exported to be accessible from GDScript
    // Controller Inputs: An external script should set these values
    [Export] public float ThrottleInput = 0.0f;
    [Export] public float SteeringInput = 0.0f;
    [Export] public float BrakeInput = 0.0f;
    [Export] public float HandbrakeInput = 0.0f;
    [Export] public float ClutchInput = 0.0f;
    /////////////////////////////////////////////////////////////////////

    [ExportGroup("Internal (Don't edit!)")]
    // Internal fields which should not be set in inspector, but need to be exported to be accessible from GDScript.
    [Export] public bool IsReady = false;
    [Export] public Vector3 LocalVelocity = Vector3.Zero;
    [Export] public Vector3 PreviousGlobalPosition = Vector3.Zero;
    [Export] public float Speed = 0.0f;
    [Export] public float MotorRpm = 0.0f;

    [Export] public float SteeringAmount = 0.0f;
    [Export] public float SteeringExponentAmount = 0.0f;
    [Export] public float TrueSteeringAmount = 0.0f;
    [Export] public float ThrottleAmount = 0.0f;
    [Export] public float BrakeAmount = 0.0f;
    [Export] public float ClutchAmount = 0.0f;
    [Export] public int CurrentGear = 0;
    [Export] public int RequestedGear = 0;
    private int _requestedGear = 0;
    [Export] public float TorqueOutput = 0.0f;
    [Export] public float ClutchTorque = 0.0f;
    [Export] public float MaxClutchTorque = 0.0f;
    [Export] public float DriveAxlesInertia = 0.0f;
    [Export] public float CompleteShiftDeltaTime = 0.0f;
    [Export] public float LastShiftDeltaTime = 0.0f;
    [Export] public float AverageDriveWheelRadius = 0.0f;
    [Export] public float CurrentTorqueSplit = 0.0f;
    [Export] public float TrueTorqueSplit = 0.0f;
    [Export] public float BrakeForce = 0.0f;
    [Export] public float MaxBrakeForce = 0.0f;
    [Export] public float HandbrakeForce = 0.0f;
    [Export] public float MaxHandbrakeForce = 0.0f;
    [Export] public bool IsBraking = false;
    [Export] public bool MotorIsRedline = false;
    [Export] public bool IsShifting = false;
    [Export] public bool IsUpShifting = false;
    [Export] public bool NeedClutch = false;
    [Export] public bool TcsActive = false;
    [Export] public bool StabilityActive = false;
    [Export] public float StabilityYawTorque = 0.0f;
    [Export] public Vector3 StabilityTorqueVector = Vector3.Zero;
    [Export] public Vector3 FrontAxlePosition = Vector3.Zero;
    [Export] public Vector3 RearAxlePosition = Vector3.Zero;
    [Export] public Vector3 VehicleInertia;

    [Export] public Array<GEVPWheel> WheelArray = new Array<GEVPWheel>();
    [Export] public Array<Axle> Axles = new Array<Axle>();
    [Export] public Axle FrontAxle;
    [Export] public Axle RearAxle;
    [Export] public Array<GEVPWheel> DriveWheels = new Array<GEVPWheel>();

    internal float DeltaTime = 0.0f;
    internal Vector3 CurrentGravity;

    public partial class Axle : GodotObject
    {
        public Array<GEVPWheel> Wheels = new Array<GEVPWheel>();
        public bool IsDriveAxle = false;
        public float Inertia = 0.0f;
        public bool Handbrake = false;
        public float BrakeBias = 0.5f;
        public float RotationSplit = 0.5f;
        public float AppliedSplit = 0.5f;
        public float TorqueVectoring = 0.0f;
        public float SuspensionCompressionLeft = 0.0f;
        public float SuspensionCompressionRight = 0.0f;
        public float TireSizeCorrection = 0.0f;
        public float DifferentialLockTorque = 0.0f;

        // Get the maximum wheel spin
        public float GetSpin()
        {
            float spin = 0.0f;
            foreach (GEVPWheel wheel in Wheels)
            {
                spin = Mathf.Max(spin, wheel.Spin);
            }
            return spin * TireSizeCorrection;
        }

        // Get the average wheel spin
        public float GetAverageSpin()
        {
            float spin = 0.0f;
            foreach (GEVPWheel wheel in Wheels)
            {
                spin += wheel.Spin;
            }
            return spin / Wheels.Count;
        }

        // Get the maximum wheel slip in the Y-axis (lateral slip)
        public float GetMaxWheelSlipY()
        {
            float slip = 0.0f;
            foreach (GEVPWheel wheel in Wheels)
            {
                slip = Mathf.Max(slip, wheel.SlipVector.Y);
            }
            return slip;
        }
    }

    public override void _Ready()
    {
        Initialize();
    }

    public override void _IntegrateForces(PhysicsDirectBodyState3D state)
    {
        CurrentGravity = state.TotalGravity;
    }

    public void Initialize()
    {
        // Check to verify that surface types are provided
        if (TireStiffnesses.Count == 0)
        {
            GD.PushError("No surface types provided for tire stiffness");
            return;
        }

        if (CoefficientOfFriction.Count == 0)
        {
            GD.PushError("No surface types provided for coefficient of friction");
            return;
        }

        if (RollingResistance.Count == 0)
        {
            GD.PushError("No surface types provided in rolling resistance");
            return;
        }

        if (LateralGripAssist.Count == 0)
        {
            GD.PushError("No surface types provided in lateral grip assist");
            return;
        }

        if (LongitudinalGripRatio.Count == 0)
        {
            GD.PushError("No surface types provided in longitudinal grip ratio");
            return;
        }

        string defaultSurface = TireStiffnesses.Keys.First();

        CenterOfMassMode = RigidBody3D.CenterOfMassModeEnum.Custom;
        Mass = VehicleMass;
        var centerOfGravity = CalculateCenterOfGravity(FrontWeightDistribution);
        centerOfGravity.Y += CenterOfGravityHeightOffset;
        CenterOfMass = centerOfGravity;
        MaxClutchTorque = MaxTorque * MaxClutchTorqueRatio;

        FrontAxle = new Axle();
        FrontAxle.Wheels.Add(FrontLeftWheel);
        FrontAxle.Wheels.Add(FrontRightWheel);
        FrontAxle.TorqueVectoring = FrontTorqueVectoring;
        FrontLeftWheel.OppositeWheel = FrontRightWheel;
        FrontLeftWheel.BeamAxle = FrontBeamAxle ? 1.0f : 0.0f;
        FrontRightWheel.OppositeWheel = FrontLeftWheel;
        FrontRightWheel.BeamAxle = FrontBeamAxle ? -1.0f : 0.0f;

        RearAxle = new Axle();
        RearAxle.Wheels.Add(RearLeftWheel);
        RearAxle.Wheels.Add(RearRightWheel);
        RearAxle.TorqueVectoring = RearTorqueVectoring;
        RearLeftWheel.OppositeWheel = RearRightWheel;
        RearLeftWheel.BeamAxle = RearBeamAxle ? 1.0f : 0.0f;
        RearRightWheel.OppositeWheel = RearLeftWheel;
        RearRightWheel.BeamAxle = RearBeamAxle ? -1.0f : 0.0f;
        RearAxle.Handbrake = true;

        Axles.Add(FrontAxle);
        Axles.Add(RearAxle);

        WheelArray.Add(FrontLeftWheel);
        WheelArray.Add(FrontRightWheel);
        WheelArray.Add(RearLeftWheel);
        WheelArray.Add(RearRightWheel);

        var maxTireRadius = Mathf.Max(FrontTireRadius, RearTireRadius);
        FrontAxle.TireSizeCorrection = maxTireRadius / FrontTireRadius;
        RearAxle.TireSizeCorrection = maxTireRadius / RearTireRadius;

        FrontAxle.DifferentialLockTorque = FrontLockingDifferentialEngageTorque;
        RearAxle.DifferentialLockTorque = RearLockingDifferentialEngageTorque;

        foreach (var wheel in WheelArray)
        {
            wheel.SurfaceType = defaultSurface;
            wheel.TireStiffnesses = TireStiffnesses;
            wheel.ContactPatch = ContactPatch;
            wheel.BrakingGripMultiplier = BrakingGripMultiplier;
            wheel.CoefficientOfFriction = CoefficientOfFriction;
            wheel.RollingResistance = RollingResistance;
            wheel.LateralGripAssist = LateralGripAssist;
            wheel.LongitudinalGripRatio = LongitudinalGripRatio;
            wheel.WheelToBodyTorqueMultiplier = WheelToBodyTorqueMultiplier;
        }

        var frontWeightPerWheel = VehicleMass * FrontWeightDistribution * 4.9f;
        var frontSpringRate = CalculateSpringRate(frontWeightPerWheel, FrontSpringLength, FrontRestingRatio);
        var frontDampingRate = CalculateDamping(frontWeightPerWheel, frontSpringRate, FrontDampingRatio);

        foreach (var wheel in FrontAxle.Wheels)
        {
            wheel.WheelMass = FrontWheelMass;
            wheel.TireRadius = FrontTireRadius;
            wheel.TireWidth = FrontTireWidth;
            wheel.SteeringRatio = FrontSteeringRatio;
            wheel.SpringLength = FrontSpringLength;
            wheel.SpringRate = frontSpringRate;
            wheel.Antiroll = frontSpringRate * FrontArbRatio;
            wheel.SlowBump = frontDampingRate * FrontBumpDampMultiplier;
            wheel.SlowRebound = frontDampingRate * FrontReboundDampMultiplier;
            wheel.FastBump = frontDampingRate * FrontBumpDampMultiplier * 0.5f;
            wheel.FastRebound = frontDampingRate * FrontReboundDampMultiplier * 0.5f;
            wheel.BumpStopMultiplier = FrontBumpStopMultiplier;
            wheel.MassOverWheel = VehicleMass * FrontWeightDistribution * 0.5f;
            wheel.AbsPulseTime = FrontAbsPulseTime;
            wheel.AbsSpinDifferenceThreshold = -Mathf.Abs(FrontAbsSpinDifferenceThreshold);
        }

        var rearWeightPerWheel = VehicleMass * (1.0f - FrontWeightDistribution) * 4.9f;
        var rearSpringRate = CalculateSpringRate(rearWeightPerWheel, RearSpringLength, RearRestingRatio);
        var rearDampingRate = CalculateDamping(rearWeightPerWheel, rearSpringRate, RearDampingRatio);

        foreach (var wheel in RearAxle.Wheels)
        {
            wheel.WheelMass = RearWheelMass;
            wheel.TireRadius = RearTireRadius;
            wheel.TireWidth = RearTireWidth;
            wheel.SteeringRatio = RearSteeringRatio;
            wheel.SpringLength = RearSpringLength;
            wheel.SpringRate = rearSpringRate;
            wheel.Antiroll = rearSpringRate * RearArbRatio;
            wheel.SlowBump = rearDampingRate * RearBumpDampMultiplier;
            wheel.SlowRebound = rearDampingRate * RearReboundDampMultiplier;
            wheel.FastBump = rearDampingRate * RearBumpDampMultiplier * 0.5f;
            wheel.FastRebound = rearDampingRate * RearReboundDampMultiplier * 0.5f;
            wheel.BumpStopMultiplier = RearBumpStopMultiplier;
            wheel.MassOverWheel = VehicleMass * (1.0f - FrontWeightDistribution) * 0.5f;
            wheel.AbsPulseTime = RearAbsPulseTime;
            wheel.AbsSpinDifferenceThreshold = -Mathf.Abs(RearAbsSpinDifferenceThreshold);
        }

        var wheelBase = RearLeftWheel.Position.Z - FrontLeftWheel.Position.Z;
        var frontTrackWidth = FrontRightWheel.Position.X - FrontLeftWheel.Position.X;
        var frontAckermann = (Mathf.Atan((wheelBase * Mathf.Tan(MaxSteeringAngle)) / (wheelBase - (frontTrackWidth * 0.5f * Mathf.Tan(MaxSteeringAngle)))) / MaxSteeringAngle) - 1.0f;
        var rearTrackWidth = RearRightWheel.Position.X - RearLeftWheel.Position.X;
        var rearAckermann = (Mathf.Atan((wheelBase * Mathf.Tan(MaxSteeringAngle)) / (wheelBase - (rearTrackWidth * 0.5f * Mathf.Tan(MaxSteeringAngle)))) / MaxSteeringAngle) - 1.0f;

        FrontLeftWheel.Ackermann = frontAckermann;
        var frontLeftWheelRotation = FrontLeftWheel.Rotation;
        frontLeftWheelRotation.Z = -FrontCamber;
        FrontLeftWheel.Rotation = frontLeftWheelRotation;
        FrontLeftWheel.Toe = -FrontToe;

        FrontRightWheel.Ackermann = -frontAckermann;
        var frontRightWheelRotation = FrontRightWheel.Rotation;
        frontRightWheelRotation.Z = FrontCamber;
        FrontRightWheel.Rotation = frontRightWheelRotation;
        FrontRightWheel.Toe = FrontToe;

        RearLeftWheel.Ackermann = rearAckermann;
        var rearLeftWheelRotation = RearLeftWheel.Rotation;
        rearLeftWheelRotation.Z = -RearCamber;
        RearLeftWheel.Rotation = rearLeftWheelRotation;
        RearLeftWheel.Toe = -RearToe;

        RearRightWheel.Ackermann = -rearAckermann;
        var rearRightWheelRotation = RearRightWheel.Rotation;
        rearRightWheelRotation.Z = RearCamber;
        RearRightWheel.Rotation = rearRightWheelRotation;
        RearRightWheel.Toe = RearToe;

        if (FrontBrakeBias < 0.0f)
        {
            var frontAxleSpringForce = CalculateAxleSpringForce(0.6f, FrontSpringLength, frontSpringRate);
            var totalSpringForce = frontAxleSpringForce + CalculateAxleSpringForce(0.4f, RearSpringLength, rearSpringRate);
            FrontBrakeBias = frontAxleSpringForce / totalSpringForce;
        }

        FrontAxle.BrakeBias = FrontBrakeBias;
        RearAxle.BrakeBias = 1.0f - FrontBrakeBias;

        foreach (var wheel in WheelArray)
        {
            wheel.Initialize();
        }

        if (FrontTorqueSplit > 0.0f || VariableTorqueSplit)
        {
            FrontAxle.IsDriveAxle = true;
        }
        if (FrontTorqueSplit < 1.0f || VariableTorqueSplit)
        {
            RearAxle.IsDriveAxle = true;
        }

        foreach (var wheel in FrontAxle.Wheels)
        {
            FrontAxle.Inertia += wheel.WheelMoment;
            if (FrontAxle.IsDriveAxle)
            {
                DriveAxlesInertia += wheel.WheelMoment;
                DriveWheels.Add(wheel);
                wheel.IsDriven = true;
                AverageDriveWheelRadius += wheel.TireRadius;
            }
        }

        foreach (var wheel in RearAxle.Wheels)
        {
            RearAxle.Inertia += wheel.WheelMoment;
            if (RearAxle.IsDriveAxle)
            {
                DriveAxlesInertia += wheel.WheelMoment;
                DriveWheels.Add(wheel);
                wheel.IsDriven = true;
                AverageDriveWheelRadius += wheel.TireRadius;
            }
        }

        AverageDriveWheelRadius /= DriveWheels.Count;
        PreviousGlobalPosition = GlobalPosition;

        CalculateBrakeForce();

        IsReady = true;
    }

    public override void _PhysicsProcess(double deltad)
    {
        var delta = (float)deltad;
        if (!IsReady)
            return;

        // For stability calculations, we need the vehicle body inertia which isn't available immediately
        if (VehicleInertia == null)
        {
            var rigidbodyInertia = PhysicsServer3D.BodyGetDirectState(GetRid()).InverseInertia.Inverse();
            if (rigidbodyInertia.IsFinite())
            {
                VehicleInertia = rigidbodyInertia * InertiaMultiplier;
                Inertia = VehicleInertia;
            }
        }

        DeltaTime += delta;
        LocalVelocity = (((GlobalTransform.Origin - PreviousGlobalPosition) / delta) * GlobalTransform.Basis).Lerp(LocalVelocity, 0.5f);
        PreviousGlobalPosition = GlobalPosition;
        Speed = LocalVelocity.Length();

        ProcessDrag();
        ProcessBraking(delta);
        ProcessSteering(delta);
        ProcessThrottle(delta);
        ProcessMotor(delta);
        ProcessClutch(delta);
        ProcessTransmission(delta);
        ProcessDrive(delta);
        ProcessForces(delta);
        ProcessStability();
    }


    public void ProcessDrag()
    {
        float drag = 0.5f * AirDensity * Mathf.Pow(Speed, 2.0f) * FrontalArea * CoefficientOfDrag;
        if (drag > 0.0f)
        {
            ApplyCentralForce(-LocalVelocity.Normalized() * drag);
        }
    }

    public void ProcessBraking(float delta)
    {
        if (BrakeInput < BrakeAmount)
        {
            BrakeAmount -= BrakingSpeed * delta;
            if (BrakeInput > BrakeAmount)
            {
                BrakeAmount = BrakeInput;
            }
        }
        else if (BrakeInput > BrakeAmount)
        {
            BrakeAmount += BrakingSpeed * delta;
            if (BrakeInput < BrakeAmount)
            {
                BrakeAmount = BrakeInput;
            }
        }

        if (BrakeAmount > 0.0f)
        {
            IsBraking = true;
        }
        else
        {
            IsBraking = false;
        }

        BrakeForce = BrakeAmount * MaxBrakeForce;
        HandbrakeForce = HandbrakeInput * MaxHandbrakeForce;
    }

    public void ProcessSteering(float delta)
    {
        bool steerAssistEngaged = false;
        float steeringSlip = GetMaxSteeringSlipAngle();

        // Adjust steering speed based on vehicle speed and max steering angle
        float steerSpeedCorrection = SteeringSpeed / (Speed * SteeringSpeedDecay) / MaxSteeringAngle;

        // If the steering input is opposite the current steering, apply countersteering speed instead
        if (Mathf.Sign(SteeringInput) != Mathf.Sign(SteeringAmount))
        {
            steerSpeedCorrection = CountersteerSpeed / (Speed * SteeringSpeedDecay);
        }

        // Check steering slip threshold and reduce steering amount if crossed.
        if (Mathf.Abs(steeringSlip) > SteeringSlipAssist)
        {
            steerAssistEngaged = true;
        }

        if (SteeringInput < SteeringAmount)
        {
            if (!steerAssistEngaged || steeringSlip < 0.0f)
            {
                SteeringAmount -= steerSpeedCorrection * delta;
                if (SteeringInput > SteeringAmount)
                {
                    SteeringAmount = SteeringInput;
                }
            }
            else
            {
                SteeringAmount += steerSpeedCorrection * delta;
                if (SteeringAmount > 0.0f)
                {
                    SteeringAmount = 0.0f;
                }
            }
        }
        else if (SteeringInput > SteeringAmount)
        {
            if (!steerAssistEngaged || steeringSlip > 0.0f)
            {
                SteeringAmount += steerSpeedCorrection * delta;
                if (SteeringInput < SteeringAmount)
                {
                    SteeringAmount = SteeringInput;
                }
            }
            else
            {
                SteeringAmount -= steerSpeedCorrection * delta;
                if (SteeringAmount < 0.0f)
                {
                    SteeringAmount = 0.0f;
                }
            }
        }

        // Steering exponent adjustment
        float steeringAdjust = Mathf.Pow(Mathf.Abs(SteeringAmount), SteeringExponent) * Mathf.Sign(SteeringAmount);

        // Correct steering toward the direction of travel for countersteer assist
        float steerCorrection = (1.0f - Mathf.Abs(steeringAdjust)) * Mathf.Clamp(Mathf.Asin(LocalVelocity.Normalized().X), -MaxSteeringAngle, MaxSteeringAngle) * CountersteerAssist;

        // Don't apply corrections at low velocity or reversing
        if (LocalVelocity.Z > -0.5f)
        {
            steerCorrection = 0.0f;
        }
        else
        {
            steerCorrection = steerCorrection / -MaxSteeringAngle;
        }

        // Keeps steering corrections from getting stuck under certain circumstances
        float steerCorrectionAmount = 1.0f;
        if (Mathf.Sign(steeringAdjust + steerCorrection) != Mathf.Sign(SteeringInput) && 1.0f - Mathf.Abs(SteeringInput) < steerCorrectionAmount)
        {
            steerCorrectionAmount = Mathf.Clamp(steerCorrectionAmount - (SteeringSpeed * delta), 0.0f, 1.0f);
        }
        else
        {
            steerCorrectionAmount = Mathf.Clamp(steerCorrectionAmount + (SteeringSpeed * delta), 0.0f, 1.0f);
        }

        steerCorrection *= steerCorrectionAmount;

        TrueSteeringAmount = Mathf.Clamp(steeringAdjust + steerCorrection, -MaxSteeringAngle, MaxSteeringAngle);

        foreach (var wheel in WheelArray)
        {
            wheel.Steer(steeringAdjust + steerCorrection, MaxSteeringAngle);
        }
    }

    public void ProcessThrottle(float delta)
    {
        float throttleDelta = ThrottleSpeed * delta;

        if (ThrottleInput < ThrottleAmount)
        {
            ThrottleAmount -= throttleDelta;
            if (ThrottleInput > ThrottleAmount)
            {
                ThrottleAmount = ThrottleInput;
            }
        }
        else if (ThrottleInput >= ThrottleAmount)
        {
            ThrottleAmount += throttleDelta;
            if (ThrottleInput < ThrottleAmount)
            {
                ThrottleAmount = ThrottleInput;
            }
        }

        // Cut throttle at redline and when shifting
        if (MotorIsRedline || IsShifting)
        {
            ThrottleAmount = 0.0f;
        }

        // Disengage clutch when shifting or below motor idle
        if (NeedClutch || IsShifting)
        {
            ClutchAmount = 1.0f;
        }
        else
        {
            ClutchAmount = ClutchInput;
        }
    }

    public void ProcessMotor(float delta)
    {
        float dragTorque = MotorRpm * MotorDrag;
        TorqueOutput = GetTorqueAtRpm(MotorRpm) * ThrottleAmount;

        // Adjust torque based on throttle input, clutch input, and motor drag
        TorqueOutput -= dragTorque * (1.0f + (ClutchAmount * (1.0f - ThrottleAmount)));

        // Prevent motor from outputting torque below idle or far beyond redline
        float newRpm = MotorRpm;
        newRpm += AngularVelocityToRpm * delta * TorqueOutput / MotorMoment;
        MotorIsRedline = false;
        if (newRpm > MaxRpm * 1.1f || newRpm <= IdleRpm)
        {
            TorqueOutput = 0.0f;
            if (newRpm > MaxRpm * 1.1f)
            {
                MotorIsRedline = true;
            }
        }

        MotorRpm += AngularVelocityToRpm * delta * (TorqueOutput - dragTorque) / MotorMoment;

        // Disengage clutch when near idle
        if (MotorRpm < IdleRpm + 100)
        {
            NeedClutch = true;
        }
        else if (newRpm > Mathf.Max(ClutchOutRpm, IdleRpm))
        {
            NeedClutch = false;
        }

        MotorRpm = Mathf.Max(MotorRpm, IdleRpm);
    }

    public void ProcessClutch(float delta)
    {
        if (CurrentGear == 0)
        {
            return;
        }

        // Formula to calculate the forces needed to keep the drivetrain and motor closely coupled
        float currentGearRatio = GetGearRatio(CurrentGear);
        float driveInertia = MotorMoment + (Mathf.Pow(Mathf.Abs(currentGearRatio), 2.0f) * GearInertia) + DriveAxlesInertia;
        float driveInertiaR = driveInertia / (currentGearRatio * currentGearRatio);
        float reactionTorque = GetDriveWheelsReactionTorque() / currentGearRatio;
        float speedDifference = (MotorRpm / AngularVelocityToRpm) - (GetDrivetrainSpin() * currentGearRatio);

        if (speedDifference < 0.0f)
        {
            speedDifference = -Mathf.Sqrt(-speedDifference);
        }

        float a = (MotorMoment * driveInertiaR * speedDifference) / delta;
        float b = MotorMoment * reactionTorque;
        float c = driveInertiaR * TorqueOutput;
        float clutchFactor = (1.0f - ClutchAmount);
        float tcsTorqueReduction = 0.0f;

        ClutchTorque = ((a - b + c) / (MotorMoment + driveInertiaR)) * clutchFactor;
        ClutchTorque = Mathf.Clamp(ClutchTorque, -MaxClutchTorque * clutchFactor, MaxClutchTorque * clutchFactor);

        // Check if traction control is needed and adjust motor speed and clutch torque if needed
        if (TractionControlMaxSlip > 0.0f)
        {
            float slipY = 0.0f;
            foreach (var axle in Axles)
            {
                slipY = Mathf.Max(slipY, axle.GetMaxWheelSlipY());
            }

            if (slipY > TractionControlMaxSlip)
            {
                tcsTorqueReduction = TorqueOutput;
                ClutchTorque = 0.0f;
                TcsActive = true;
            }
            else
            {
                TcsActive = false;
            }
        }

        float clutchReactionTorque = ClutchTorque + tcsTorqueReduction;
        float newRpm = MotorRpm - ((AngularVelocityToRpm * delta * clutchReactionTorque) / MotorMoment);
        if (newRpm < IdleRpm)
        {
            newRpm = IdleRpm;
        }

        if (newRpm < IdleRpm + 100)
        {
            NeedClutch = true;
        }
        else if (newRpm > Mathf.Max(ClutchOutRpm, IdleRpm))
        {
            NeedClutch = false;
        }

        if (newRpm > MaxRpm * 1.1f)
        {
            newRpm = MaxRpm * 1.1f;
        }

        MotorRpm = newRpm;
    }


    public void ProcessTransmission(float delta)
    {
        if (IsShifting)
        {
            if (DeltaTime > CompleteShiftDeltaTime)
            {
                CompleteShift();
            }
            return;
        }

        // For automatic transmissions to determine when to shift, the current wheel speed and
        // what the wheel speed would be without slip are used. This allows vehicles to spin the
        // tires without immediately shifting to the next gear.

        if (AutomaticTransmission)
        {
            bool reversing = false;
            float idealWheelSpin = Speed / AverageDriveWheelRadius;
            float drivetrainSpin = GetDrivetrainSpin();
            float realWheelSpin = drivetrainSpin * GetGearRatio(CurrentGear);
            float currentIdealGearRpm = GearRatios[GetGear(CurrentGear - 1)] * FinalDrive * idealWheelSpin * AngularVelocityToRpm;
            float currentRealGearRpm = realWheelSpin * AngularVelocityToRpm;

            if (CurrentGear == -1)
            {
                reversing = true;
            }

            if (!reversing)
            {
                float previousGearRpm = 0.0f;
                if (CurrentGear - 1 > 0)
                {
                    previousGearRpm = GetGearRatio(CurrentGear - 1) * Mathf.Max(drivetrainSpin, idealWheelSpin) * AngularVelocityToRpm;
                }

                if (CurrentGear < GearRatios.Count)
                {
                    if (CurrentGear > 0)
                    {
                        if (currentIdealGearRpm > MaxRpm)
                        {
                            if (DeltaTime - LastShiftDeltaTime > ShiftTime)
                            {
                                Shift(1);
                            }
                        }
                        if (currentIdealGearRpm > MaxRpm * 0.8f && currentRealGearRpm > MaxRpm)
                        {
                            if (DeltaTime - LastShiftDeltaTime > ShiftTime)
                            {
                                Shift(1);
                            }
                        }
                    }
                    else if (CurrentGear == 0 && MotorRpm > Mathf.Max(ClutchOutRpm, IdleRpm))
                    {
                        Shift(1);
                    }
                }

                if (CurrentGear - 1 > 0)
                {
                    if (CurrentGear > 1 && previousGearRpm < 0.75f * MaxRpm)
                    {
                        if (DeltaTime - LastShiftDeltaTime > ShiftTime)
                        {
                            Shift(-1);
                        }
                    }
                }
            }

            if (Mathf.Abs(CurrentGear) <= 1 && BrakeInput > 0.75f)
            {
                if (!reversing)
                {
                    if (Speed < 1.0f || LocalVelocity.Z > 0.0f)
                    {
                        if (DeltaTime - LastShiftDeltaTime > ShiftTime)
                        {
                            Shift(-1);
                        }
                    }
                }
                else
                {
                    if (Speed < 1.0f || LocalVelocity.Z < 0.0f)
                    {
                        if (DeltaTime - LastShiftDeltaTime > ShiftTime)
                        {
                            Shift(1);
                        }
                    }
                }
            }
        }
    }

    public void ProcessDrive(float delta)
    {
        float currentGearRatio = GetGearRatio(CurrentGear);
        float driveTorque = 0.0f;
        float driveInertia = MotorMoment + Mathf.Pow(currentGearRatio, 2) * GearInertia;
        bool isSlipping = GetIsAWheelSlipping();

        if (CurrentGear != 0)
        {
            driveTorque = ClutchTorque * currentGearRatio;
        }

        // Check for slip and adjust variable torque split
        if (VariableTorqueSplit)
        {
            if (isSlipping && ThrottleAmount > 0.1f)
            {
                CurrentTorqueSplit = Mathf.Clamp(CurrentTorqueSplit + (delta / VariableSplitSpeed), 0.0f, 1.0f);
            }
            else
            {
                CurrentTorqueSplit = Mathf.Clamp(CurrentTorqueSplit - (delta / VariableSplitSpeed), 0.0f, 1.0f);
            }
        }

        // Same formula to keep the motor and drivetrain coupled, but to keep the front and rear axles
        // coupled. Modified to allow for a torque split.
        TrueTorqueSplit = Mathf.Lerp(FrontTorqueSplit, FrontVariableSplit, CurrentTorqueSplit);
        Axle axleA = FrontAxle;
        Axle axleB = RearAxle;

        if (TrueTorqueSplit <= 0.5f)
        {
            axleA = RearAxle;
            axleB = FrontAxle;
        }

        float axleDifference = axleA.GetSpin() - axleB.GetSpin();
        float a = (axleA.Inertia * axleB.Inertia * axleDifference) / delta;
        float b = axleA.Inertia;
        float c = axleB.Inertia * driveTorque;
        float transferTorque = (a - b + c) / (axleA.Inertia + axleB.Inertia);
        transferTorque = Mathf.Clamp(transferTorque, -Mathf.Abs(driveTorque), Mathf.Abs(driveTorque)) * (1.0f - Mathf.Abs((0.5f - TrueTorqueSplit) * 2.0f));
        float transferTorque2 = driveTorque - transferTorque;

        ProcessAxleDrive(axleB, transferTorque, driveInertia, delta);
        ProcessAxleDrive(axleA, transferTorque2, driveInertia, delta);
    }

    public void ProcessAxleDrive(Axle axle, float torque, float driveInertia, float delta)
    {
        if (!axle.IsDriveAxle)
        {
            torque = 0.0f;
            driveInertia = 0.0f;
        }

        bool allowAbs = true;

        // If the handbrake is engaged, disable the antilock brakes
        if (axle.Handbrake)
        {
            BrakeForce += HandbrakeForce;
            allowAbs = false;
        }

        // If enough torque is applied to the axle, lock to wheel speeds and add torque vectoring
        if (axle.IsDriveAxle && axle.DifferentialLockTorque >= 0.0f)
        {
            if (Mathf.Abs(torque) > axle.DifferentialLockTorque)
            {
                axle.RotationSplit = 0.5f + (axle.TorqueVectoring * -SteeringInput);
                float coupleSpin = axle.GetAverageSpin();
                axle.Wheels[0].Spin = coupleSpin * axle.RotationSplit * 2.0f;
                axle.Wheels[1].Spin = coupleSpin * (1.0f - axle.RotationSplit) * 2.0f;
                axle.RotationSplit = (axle.RotationSplit * 2.0f) - 1.0f;
            }
            else if (torque != 0.0f)
            {
                float leftReactionTorqueRatio = -Mathf.Abs(axle.Wheels[0].GetReactionTorque() / torque);
                float rightReactionTorqueRatio = Mathf.Abs(axle.Wheels[1].GetReactionTorque() / torque);
                axle.RotationSplit = Mathf.Max(axle.RotationSplit, leftReactionTorqueRatio);
                axle.RotationSplit = Mathf.Min(axle.RotationSplit, rightReactionTorqueRatio);
            }
        }

        float rotationSum = 0.0f;
        float split = (axle.RotationSplit + 1.0f) * 0.5f;
        axle.AppliedSplit = axle.RotationSplit;
        rotationSum += axle.Wheels[0].ProcessTorque(torque * split, driveInertia, BrakeForce * 0.5f * axle.BrakeBias, allowAbs, delta);
        rotationSum += axle.Wheels[1].ProcessTorque(torque * (1.0f - split), driveInertia, BrakeForce * 0.5f * axle.BrakeBias, allowAbs, delta);
        axle.RotationSplit = Mathf.Clamp(rotationSum, -1.0f, 1.0f);
    }

    public void ProcessForces(float delta)
    {
        // Spring compression values are kept for antiroll bar calculations
        foreach (var axle in Axles)
        {
            float previousCompressionLeft = axle.SuspensionCompressionLeft;
            axle.SuspensionCompressionLeft = axle.Wheels[0].ProcessForces(axle.SuspensionCompressionRight, IsBraking, delta);
            axle.SuspensionCompressionRight = axle.Wheels[1].ProcessForces(previousCompressionLeft, IsBraking, delta);
        }
    }

    public void ProcessStability()
    {
        bool isStabilityOn = false;

        // Calculates the angle of the vehicle in relation to the direction of travel
        // and applies necessary stabilizing forces.
        if (EnableStability)
        {
            StabilityYawTorque = 0.0f;
            Vector2 planeXZ = new Vector2(LocalVelocity.X, LocalVelocity.Z);

            if (planeXZ.Y < 0 && planeXZ.Length() > 1.0f)
            {
                planeXZ = planeXZ.Normalized();
                float yawAngle = 1.0f - Mathf.Abs(planeXZ.Dot(Vector2.Up));
                if (yawAngle > StabilityYawEngageAngle && Mathf.Sign(AngularVelocity.Y) == Mathf.Sign(planeXZ.X))
                {
                    StabilityYawTorque = (yawAngle - StabilityYawEngageAngle) * StabilityYawStrength;
                    StabilityYawTorque *= VehicleInertia.Y * Mathf.Clamp(Mathf.Abs(AngularVelocity.Y) - 0.5f, 0.0f, 1.0f);
                }
            }

            StabilityTorqueVector = Vector3.Zero;
            if (GetWheelContactCount() < 3)
            {
                StabilityTorqueVector = (GlobalTransform.Basis.Y.Cross(Vector3.Up) * VehicleInertia * StabilityUprightSpring) + (-AngularVelocity * StabilityUprightDamping);
                ApplyTorque(StabilityTorqueVector);
            }
            else
            {
                StabilityYawTorque *= StabilityYawGroundMultiplier;
            }

            if (StabilityYawTorque != 0.0f)
            {
                isStabilityOn = true;
                StabilityYawTorque *= Mathf.Sign(-LocalVelocity.X);
                ApplyTorque(GlobalTransform.Basis.Y * StabilityYawTorque);
            }
        }

        StabilityActive = isStabilityOn;
    }


    public void ManualShift(int count)
    {
        if (!AutomaticTransmission)
        {
            Shift(count);
        }
    }

    public void Shift(int count)
    {
        if (IsShifting)
        {
            return;
        }

        // Handles gear change requests and timings
        RequestedGear = CurrentGear + count;

        if (RequestedGear <= GearRatios.Count && RequestedGear >= -1)
        {
            if (CurrentGear == 0)
            {
                CompleteShift();
            }
            else
            {
                CompleteShiftDeltaTime = DeltaTime + ShiftTime;
                ClutchAmount = 1.0f;
                IsShifting = true;
                if (count > 0)
                {
                    IsUpShifting = true;
                }
            }
        }
    }

    public void CompleteShift()
    {
        // Called when it is time to complete a shift in progress
        if (CurrentGear == -1)
        {
            BrakeAmount = 0.0f;
        }

        if (RequestedGear < CurrentGear)
        {
            float wheelSpin = Speed / AverageDriveWheelRadius;
            float requestedGearRpm = GearRatios[GetGear(RequestedGear - 1)] * FinalDrive * wheelSpin * AngularVelocityToRpm;
            MotorRpm = Mathf.Lerp(MotorRpm, requestedGearRpm, 0.5f);
        }

        CurrentGear = RequestedGear;
        LastShiftDeltaTime = DeltaTime;
        IsShifting = false;
        IsUpShifting = false;
    }

    public int GetWheelContactCount()
    {
        int contactCount = 0;
        foreach (GEVPWheel wheel in WheelArray)
        {
            if (wheel.IsColliding())
            {
                contactCount++;
            }
        }
        return contactCount;
    }

    public bool GetIsAWheelSlipping()
    {
        bool isSlipping = false;
        foreach (GEVPWheel wheel in DriveWheels)
        {
            if (!wheel.LimitSpin)
            {
                isSlipping = true;
            }
        }
        return isSlipping;
    }

    public float GetDrivetrainSpin()
    {
        if (DriveWheels.Count == 0)
        {
            return 0.0f;
        }

        float driveSpin = 0.0f;
        foreach (GEVPWheel wheel in DriveWheels)
        {
            driveSpin += wheel.Spin;
        }

        return driveSpin / DriveWheels.Count;
    }

    public float GetDriveWheelsReactionTorque()
    {
        float reactionTorque = 0.0f;
        foreach (GEVPWheel wheel in DriveWheels)
        {
            reactionTorque += wheel.ForceVector.Y * wheel.TireRadius;
        }
        return reactionTorque;
    }

    public float GetGearRatio(int gear)
    {
        if (gear > 0)
        {
            return GearRatios[GetGear(gear - 1)] * FinalDrive;
        }
        else if (gear == -1)
        {
            return -ReverseRatio * FinalDrive;
        }
        else
        {
            return 0.0f;
        }
    }

    public float GetTorqueAtRpm(float lookupRpm)
    {
        float rpmFactor = Mathf.Clamp(lookupRpm / MaxRpm, 0.0f, 1.0f);
        float torqueFactor = TorqueCurve.SampleBaked(rpmFactor);
        return torqueFactor * MaxTorque;
    }

    public float GetMaxSteeringSlipAngle()
    {
        float steeringSlip = 0.0f;
        foreach (GEVPWheel wheel in FrontAxle.Wheels)
        {
            if (Mathf.Abs(steeringSlip) < Mathf.Abs(wheel.SlipVector.X))
            {
                steeringSlip = wheel.SlipVector.X;
            }
        }
        return steeringSlip;
    }

    public float CalculateAverageTireFriction(float weight, string surface)
    {
        float friction = 0.0f;
        foreach (GEVPWheel wheel in WheelArray)
        {
            friction += wheel.GetFriction(weight / WheelArray.Count, surface);
        }
        return friction;
    }
    public void CalculateBrakeForce()
    {
        float friction = CalculateAverageTireFriction(VehicleMass * 9.8f, "Road");
        MaxBrakeForce = ((friction * BrakingGripMultiplier) * AverageDriveWheelRadius) / WheelArray.Count;
        MaxHandbrakeForce = ((friction * BrakingGripMultiplier * 0.05f) / AverageDriveWheelRadius);
    }

    public Vector3 CalculateCenterOfGravity(float frontDistribution)
    {
        FrontAxlePosition = FrontLeftWheel.Position.Lerp(FrontRightWheel.Position, 0.5f);
        RearAxlePosition = RearLeftWheel.Position.Lerp(RearRightWheel.Position, 0.5f);
        return RearAxlePosition.Lerp(FrontAxlePosition, frontDistribution);
    }

    public float CalculateSpringRate(float weight, float springLength, float restingRatio)
    {
        float correctedRestingRatio = (springLength * restingRatio) / springLength;
        float targetCompression = springLength * correctedRestingRatio * 1000.0f;
        return weight / targetCompression;
    }

    public float CalculateDamping(float weight, float springRate, float dampingRatio)
    {
        return dampingRatio * 2.0f * Mathf.Sqrt(springRate * weight) * 0.01f;
    }

    public float CalculateAxleSpringForce(float compression, float springLength, float springRate)
    {
        return springLength * compression * 1000.0f * springRate * 2.0f;
    }

    
    private int GetGear(int gear)
    {
        gear = gear < 0 ? GearRatios.Count - 1 : gear;
        return gear;
    }

}
