#include "HelicopterPawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "PhysicsEngine/PhysicsThrusterComponent.h"

// Sets default values
AHelicopterPawn::AHelicopterPawn()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	// Structure to hold one - time initialization
	struct FConstructorStatics
	{
		ConstructorHelpers::FObjectFinderOptional<UStaticMesh> PlaneMesh;
		FConstructorStatics()
			: PlaneMesh(TEXT("/Game/Flying/AH-64.AH-64"))
		{
		}
	};
	static FConstructorStatics ConstructorStatics;
	////Add these 2 lines below
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Static Mesh"));
	Mesh->SetStaticMesh(ConstructorStatics.PlaneMesh.Get());	// Set static mesh
	Mesh->SetRelativeLocation({0, 0, -200});
	SetRootComponent(Mesh);

	

	/*
	struct FConstructorStatics
	{
		ConstructorHelpers::FObjectFinderOptional<USkeletalMesh> PlaneMesh;
		FConstructorStatics()
			: PlaneMesh(TEXT("/Game/VigilanteContent/Vehicles/West_Heli_AH64D/SK_West_Heli_AH64D.SK_West_Heli_AH64D"))
		{
		}
	};
	static FConstructorStatics ConstructorStatics;
	Mesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("Skeletal Mesh"));
	Mesh->SetSkeletalMesh(ConstructorStatics.PlaneMesh.Get());	// Set static mesh
	SetRootComponent(Mesh);
	*/

	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("Spring Arm"));
	SpringArm->AttachToComponent(Mesh, FAttachmentTransformRules::KeepWorldTransform);

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepWorldTransform);

	PhysicsThruster = CreateDefaultSubobject<UPhysicsThrusterComponent>(TEXT("Physics Thruster"));
	PhysicsThruster->AttachToComponent(Mesh, FAttachmentTransformRules::KeepWorldTransform);

	//Not setting mesh asset in here, 
	//because don't want to deal with having mesh asset always in a predetermined location
	Mesh->SetSimulatePhysics(true);
	Mesh->SetLinearDamping(1);
	Mesh->SetAngularDamping(1);

	//Not setting thruster location, because we don't know what mesh will be set
	PhysicsThruster->SetRelativeRotation(FRotator(-90, 0, 0));
	PhysicsThruster->bAutoActivate = true;

	SpringArm->TargetArmLength = 1800;
	SpringArm->SetRelativeRotation(FRotator(-20, 0, 0));
	SpringArm->bDoCollisionTest = false;
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;

	//Put in here just for testing to automatically possess pawn
	this->AutoPossessPlayer = EAutoReceiveInput::Player0;
}

// Called when the game starts or when spawned
void AHelicopterPawn::BeginPlay()
{
	Super::BeginPlay();
	
	FVector ThrusterCenterOfMassDifference = PhysicsThruster->GetComponentLocation() - Mesh->GetCenterOfMass();
	FRotator InvertedMeshRotation = Mesh->GetComponentRotation().GetInverse();
	FVector CenterOfMassActualOffset = InvertedMeshRotation.RotateVector(ThrusterCenterOfMassDifference);
	FVector CenterOfMassWantedOffset = FVector(CenterOfMassActualOffset.X, CenterOfMassActualOffset.Y, 0);
	Mesh->SetCenterOfMass(CenterOfMassWantedOffset, FName());
}

// Called every frame
void AHelicopterPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (GEngine) {
		GEngine->AddOnScreenDebugMessage(0, 1, FColor::Red, FString::Printf(TEXT("Thrust %f %s"), PhysicsThruster->ThrustStrength, *GetActorLocation().ToString()));
	}
}

// Called to bind functionality to input
void AHelicopterPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("LookUp", this, &AHelicopterPawn::MouseUp);
	//PlayerInputComponent->BindAxis("LookRight", this, &AHelicopterPawn::MouseRight);


	PlayerInputComponent->BindAxis("LookRight", this, &AHelicopterPawn::RotateRight);


	PlayerInputComponent->BindAxis("MoveUp", this, &AHelicopterPawn::MoveUp);
	PlayerInputComponent->BindAxis("MoveForward", this, &AHelicopterPawn::TiltForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &AHelicopterPawn::TiltRight);

	//PlayerInputComponent->BindAxis("MoveForward", this, &AHelicopterPawn::TiltForward);
	//PlayerInputComponent->BindAxis("MoveRight", this, &AHelicopterPawn::TiltRight);
}

void AHelicopterPawn::MouseRight(float Value)
{
	SpringArm->AddRelativeRotation(FRotator(0, Value, 0));
}

void AHelicopterPawn::MouseUp(float Value)
{
	SpringArm->AddRelativeRotation(FRotator(Value, 0, 0));
}

void AHelicopterPawn::MoveForward(float Value)
{
	bool bHasInput = !FMath::IsNearlyEqual(Value, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("MoveForward %f"), Value));
	}

	Mesh->AddForce(GetActorForwardVector() * Value * 1000);
}

void AHelicopterPawn::MoveRight(float Value)
{
	bool bHasInput = !FMath::IsNearlyEqual(Value, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("MoveRight %f"), Value));
	}

	Mesh->AddForce(GetActorRightVector() * Value * 1000);
}

void AHelicopterPawn::MoveUp(float Value)
{
	bool bHasInput = !FMath::IsNearlyEqual(Value, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("MoveUp %f"), Value));
	}

	//float DesiredUpForce = Value * VariableUpForce + ConstantUpForce;
	//PhysicsThruster->ThrustStrength = DesiredUpForce / GetActorUpVector().Z * Mesh->GetMass();

	Mesh->AddForce(GetActorUpVector() * Value * 1000);
}

void AHelicopterPawn::RotateRight(float Value)
{
	Mesh->AddTorqueInDegrees(GetActorUpVector() * Value * YawRotationSpeed, FName(), true);
}

void AHelicopterPawn::TiltForward(float Value)
{
	bool bHasInput = !FMath::IsNearlyEqual(Value, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("TiltForward %f"), Value));
	}

	float DesiredAngle = Value * DesiredTiltAngle + Mesh->GetComponentRotation().Pitch;
	float ClampedValue = FMath::Clamp(DesiredAngle, -TiltSpeedClampBound, TiltSpeedClampBound);
	Mesh->AddTorqueInDegrees(GetActorRightVector() * ClampedValue * TiltingSpeed, FName(), true);
}

void AHelicopterPawn::TiltRight(float Value)
{
	bool bHasInput = !FMath::IsNearlyEqual(Value, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("TiltRight %f"), Value));
	}

	float DesiredAngle = Value * -DesiredTiltAngle + Mesh->GetComponentRotation().Roll;
	float ClampedValue = FMath::Clamp(DesiredAngle, -TiltSpeedClampBound, TiltSpeedClampBound);
	Mesh->AddTorqueInDegrees(GetActorForwardVector() * ClampedValue * TiltingSpeed, FName(), true);
}

void AHelicopterPawn::SetDesiredTiltAngle(float Value)
{
	DesiredTiltAngle = FMath::Clamp(Value, -60.f, 60.f);
}

float AHelicopterPawn::GetDesiredTiltAngle()
{
	return DesiredTiltAngle;
}

void AHelicopterPawn::SetTiltSpeedClampBound(float Value)
{
	TiltSpeedClampBound = Value > 0 ? Value : 0;
}

float AHelicopterPawn::GetTiltSpeedClampBound()
{
	return TiltSpeedClampBound;
}

void AHelicopterPawn::SetTiltingSpeed(float Value)
{
	TiltingSpeed = Value > 0 ? Value : 0;
}

float AHelicopterPawn::GetTiltingSpeed()
{
	return TiltingSpeed;
}