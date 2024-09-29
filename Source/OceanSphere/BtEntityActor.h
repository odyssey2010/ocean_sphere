// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "GeoReferencingSystem.h"
#include "BtEntityActor.generated.h"

class AGeoReferencingSystem;

UCLASS()
class OCEANSPHERE_API ABtEntityActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ABtEntityActor();

protected:
	virtual void OnConstruction(const FTransform&) override;

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void PostLoad() override;

	void build();

	void update(float DeltaTime);

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	UProceduralMeshComponent* Mesh{ nullptr };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	FGeographicCoordinates GeoPos{ 52.07, 24.7474, 0.0 };

	UMaterial* Material{ nullptr };

	AGeoReferencingSystem* GeoRef{ nullptr };
};
