// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "BtOceanSphereActor.generated.h"

class AGeoReferencingSystem;
struct SphereBuilder;

UCLASS()
class OCEANSPHERE_API ABtOceanSphereActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ABtOceanSphereActor();

protected:
	virtual void OnConstruction(const FTransform&) override;

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void PostLoad() override;

	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	void LoadHeightTexture();

#if WITH_EDITOR
	/** If true, actor is ticked even if TickType==LEVELTICK_ViewportsOnly	 */
	virtual bool ShouldTickIfViewportsOnly() const override;

	virtual void PostEditChangeProperty(FPropertyChangedEvent& event) override;
#endif

	void build();

	void build_test();

	void update_origin(float DeltaTime);

	void update_view(float DeltaTime);

    void build_test_view(double ViewLat, double ViewLon, double ViewAlt);

	void check_geo_coords();

public:
	UPROPERTY(Transient, NonTransactional, EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	UProceduralMeshComponent* OceanMesh{ nullptr };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	UMaterial* EarthMaterial{ nullptr };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	UMaterial* ColorMaterial{ nullptr };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	UTexture2D* HeightTexture{ nullptr };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	double TestLatitude{ 0 };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	double TestLongitude{ 0 };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean")
	double OceanHeight{ 0.0 };


	UFUNCTION(CallInEditor, Category = "Ocean")
	void TestUpdate();
	
	AGeoReferencingSystem* GeoRef{ nullptr };

	SphereBuilder* SB{ nullptr };

	struct { uint16 Width, Height; } ElevDataSize;
	TArray<uint8> ElevData;

	bool is_check_geo_coord = false;

	int32 TestSectionIndex{ 1 };
};
