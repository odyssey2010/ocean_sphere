// Copyright Epic Games, Inc. All Rights Reserved.

#include "FighterPawn.h"
#include "UObject/ConstructorHelpers.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InputComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"

AFighterPawn::AFighterPawn()
{
	// Structure to hold one-time initialization
	struct FConstructorStatics
	{
		ConstructorHelpers::FObjectFinderOptional<UStaticMesh> PlaneMesh;
		FConstructorStatics()
			: PlaneMesh(TEXT("/Game/Flying/Meshes/UFO.UFO"))
		{
		}
	};
	static FConstructorStatics ConstructorStatics;

	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootTransform"));
	
	// Create body component
	BodyTransform = CreateDefaultSubobject<USceneComponent>(TEXT("BodyTransform"));
	BodyTransform->SetupAttachment(RootComponent);


	// Create static mesh component
	PlaneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PlaneMesh"));
	PlaneMesh->SetStaticMesh(ConstructorStatics.PlaneMesh.Get());	// Set static mesh
	PlaneMesh->SetupAttachment(BodyTransform);


	// Create a spring arm component
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(RootComponent);	// Attach SpringArm to RootComponent
	SpringArm->TargetArmLength = 160.0f; // The camera follows at this distance behind the character	
	SpringArm->SocketOffset = FVector(0.f,0.f,60.f);
	SpringArm->bEnableCameraLag = false;	// Do not allow camera to lag
	SpringArm->CameraLagSpeed = 15.f;

	// Create camera component 
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
	Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);	// Attach the camera
	Camera->bUsePawnControlRotation = false; // Don't rotate camera with controller

	// Set handling parameters
	Acceleration = 500.0;
	TurnSpeed = 50.f;
	MaxSpeed = 400000.f;
	MinSpeed = 500.f;

	ForwardSpeed = 50000.0;
	RightSpeed = 50000.0;
	UpSpeed = 10000.0;

	CurrentForwardSpeed = 0.0;
	CurrentRightSpeed = 0.0;
	CurrentUpSpeed = 0.0;

	CurrentPitchSpeed = 0.0;
	CurrentYawSpeed = 0.0;
	CurrentRollSpeed = 0.0;
}

void AFighterPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
	Super::NotifyHit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);

	// Deflect along the surface when we collide.
	FRotator CurrentRotation = GetActorRotation();
	SetActorRotation(FQuat::Slerp(CurrentRotation.Quaternion(), HitNormal.ToOrientationQuat(), 0.025f));
}


void AFighterPawn::SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent)
{
    // Check if PlayerInputComponent is valid (not NULL)
	check(PlayerInputComponent);

	// Bind our control axis' to callback functions
	PlayerInputComponent->BindAxis("Thrust", this, &AFighterPawn::ThrustInput);
	PlayerInputComponent->BindAxis("LookUp", this, &AFighterPawn::LookUpInput);
	PlayerInputComponent->BindAxis("LookRight", this, &AFighterPawn::LookRightInput);
	PlayerInputComponent->BindAxis("MoveForward", this, &AFighterPawn::MoveForwardInput);
	PlayerInputComponent->BindAxis("MoveUp", this, &AFighterPawn::MoveUpInput);
	PlayerInputComponent->BindAxis("MoveRight", this, &AFighterPawn::MoveRightInput);
}

void AFighterPawn::Tick(float DeltaSeconds)
{
	const FVector LocalMove{
		CurrentForwardSpeed * DeltaSeconds,
		CurrentRightSpeed * DeltaSeconds,
		CurrentUpSpeed * DeltaSeconds
	};

	// Move plan forwards (with sweep so we stop when we collide with things)
	bool CollisionSweep = false;
	AddActorLocalOffset(LocalMove, CollisionSweep);

	// Calculate change in rotation this frame
	FRotator DeltaRotation(0,0,0);
	DeltaRotation.Pitch = CurrentPitchSpeed * DeltaSeconds;
	DeltaRotation.Yaw = CurrentYawSpeed * DeltaSeconds;
	//DeltaRotation.Roll = CurrentRollSpeed * DeltaSeconds;
	AddActorLocalRotation(DeltaRotation);

	FRotator BodyDelta = FRotator::ZeroRotator;
	BodyDelta.Roll = CurrentRollSpeed * DeltaSeconds;
	AddActorLocalRotation(BodyDelta);


	if (GEngine) {
		GEngine->AddOnScreenDebugMessage(0, 1, FColor::Red, FString::Printf(TEXT("Acc:%f M:%s"), Acceleration, *LocalMove.ToString()));
	}

	// Call any parent class Tick implementation
	Super::Tick(DeltaSeconds);
}

void AFighterPawn::UpdateRoll(float Val, float SpeedMagitude)
{
	// Is there any left/right input?
	const bool bIsMoving = FMath::Abs(Val) > 0.2f;

	// If turning, yaw value is used to influence roll
	// If not turning, roll to reverse current roll value.
	float TargetRollSpeed = bIsMoving ? (SpeedMagitude * 0.5f) : (GetActorRotation().Roll * -2.f);

	// Smoothly interpolate roll speed
	CurrentRollSpeed = FMath::FInterpTo(CurrentRollSpeed, TargetRollSpeed, GetWorld()->GetDeltaSeconds(), 2.f);
}

void AFighterPawn::LookUpInput(float Val)
{
	bool bHasInput = !FMath::IsNearlyEqual(Val, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("LookUp %f"), Val));
	}

	// Target pitch speed is based in input
	float TargetPitchSpeed = (Val * TurnSpeed * 1.f);

	// When steering, we decrease pitch slightly
	TargetPitchSpeed += (FMath::Abs(CurrentYawSpeed) * -0.2f);

	// Smoothly interpolate to target pitch speed
	CurrentPitchSpeed = FMath::FInterpTo(CurrentPitchSpeed, TargetPitchSpeed, GetWorld()->GetDeltaSeconds(), 2.f);

	CurrentPitchSpeed = FMath::Clamp(CurrentPitchSpeed, -80, 90);
}

void AFighterPawn::LookRightInput(float Val)
{
	bool bHasInput = !FMath::IsNearlyEqual(Val, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("LookRight %f"), Val));
	}

	// Target yaw speed is based on input
	float TargetYawSpeed = (Val * TurnSpeed);

	// Smoothly interpolate to target yaw speed
	CurrentYawSpeed = FMath::FInterpTo(CurrentYawSpeed, TargetYawSpeed, GetWorld()->GetDeltaSeconds(), 2.f);

	UpdateRoll(Val, Val * 100);
}

void AFighterPawn::ThrustInput(float Val)
{
	/*
	// Is there any input?
	bool bHasInput = !FMath::IsNearlyEqual(Val, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("Thrust %f"), Val));
	}

	float TargetForwardSpeed = (Val * ForwardSpeed * 1.f);
	TargetForwardSpeed += (FMath::Abs(CurrentForwardSpeed) * -0.2f);
	CurrentForwardSpeed = FMath::FInterpTo(CurrentForwardSpeed, TargetForwardSpeed, GetWorld()->GetDeltaSeconds(), 2.f);
	*/
}

void AFighterPawn::MoveForwardInput(float Val)
{
	// Is there any input?
	bool bHasInput = !FMath::IsNearlyEqual(Val, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("MoveForward %f"), Val));
	}

	float TargetForwardSpeed = (Val * ForwardSpeed * 1.f);
	TargetForwardSpeed += (FMath::Abs(CurrentForwardSpeed) * -0.2f);
	CurrentForwardSpeed = FMath::FInterpTo(CurrentForwardSpeed, TargetForwardSpeed, GetWorld()->GetDeltaSeconds(), 2.f);
}

void AFighterPawn::MoveRightInput(float Val)
{
	bool bHasInput = !FMath::IsNearlyEqual(Val, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("MoveRight %f"), Val));
	}


	float TargetRightSpeed = (Val * RightSpeed * 1.f);
	TargetRightSpeed += (FMath::Abs(CurrentRightSpeed) * -0.2f);
	CurrentRightSpeed = FMath::FInterpTo(CurrentRightSpeed, TargetRightSpeed, GetWorld()->GetDeltaSeconds(), 2.f);

	UpdateRoll(Val, Val * 100);
}

void AFighterPawn::MoveUpInput(float Val)
{
	bool bHasInput = !FMath::IsNearlyEqual(Val, 0.f);
	if (bHasInput) {
		GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red, FString::Printf(TEXT("MoveUp %f"), Val));
	}

	float TargetUpSpeed = (Val * UpSpeed * 1.f);
	TargetUpSpeed += (FMath::Abs(CurrentUpSpeed) * -0.2f);
	CurrentUpSpeed = FMath::FInterpTo(CurrentUpSpeed, TargetUpSpeed, GetWorld()->GetDeltaSeconds(), 2.f);
}
