// Fill out your copyright notice in the Description page of Project Settings.


#include "BtEntityActor.h"
#include "GeoReferencingSystem.h"

// Sets default values
ABtEntityActor::ABtEntityActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	RootComponent = CreateDefaultSubobject<USceneComponent>("Root");

	Mesh = CreateDefaultSubobject<UProceduralMeshComponent>("Mesh");
	Mesh->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	static FString Path2 = TEXT("/Game/M_Colored");
	static ConstructorHelpers::FObjectFinder<UMaterial> MaterialLoader2(*Path2);
	if (MaterialLoader2.Succeeded())
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to load material at path: %s"), *Path2);

		Material = MaterialLoader2.Object;

		Mesh->SetMaterial(0, Material);
	}
}

void ABtEntityActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	build();

	update(0);
}

// Called when the game starts or when spawned
void ABtEntityActor::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ABtEntityActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	//update(DeltaTime);
}

void ABtEntityActor::PostLoad()
{
	Super::PostLoad();

}

inline void ECEF_TO_UNREAL(FVector& v) {
	v.X *= 100.0;
	v.Y *= -100.0;
	v.Z *= 100.0;
}

inline void UNREAL_TO_ECEF(FVector& v) {
	v.X *= 0.01;
	v.Y *= -0.01;
	v.Z *= 0.01;
}

extern void LLA_TO_ECEF(double lat, double lon, double alt, double* x, double* y, double* z);
extern void ECEF_TO_ENU(double lat, double lon, double x, double y, double z, double xr, double yr, double zr, double* e, double* n, double* u);

void ABtEntityActor::build()
{
	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<FColor> Colors;
	TArray<FVector2D> UV0;
	TArray<int32> Triangles;

	double width = 10000;
	double length = 100000;

	Vertices.Add({ 0, 0, 0 });
	Vertices.Add({ -width, 0, length });
	Vertices.Add({ width, 0, length });
	Vertices.Add({ 0, -width, length });
	Vertices.Add({ 0, width, length });

	Colors.Append({ FColor::White, FColor::Red, FColor::Red, FColor::Red, FColor::Red });

	Triangles.Append({ 0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1 });

	Mesh->CreateMeshSection(0, Vertices, Triangles, {}, {}, Colors, {}, false);
}

void ABtEntityActor::update(float DeltaTime)
{
	if (GeoRef == nullptr) {
		GeoRef = AGeoReferencingSystem::GetGeoReferencingSystem(GetWorld());
	}

	if (GeoRef) {
		// +Translation{ X = -19524233.088456511 Y = -8386185.1613464300 Z = -354162.71293349564 }
		// Tangent update ship, -19534118.062786 - 8298986.588209 - 353318.558121
				
		//GeoPos = { 52.07, 24.7474, 0.0 };
		FTransform Transform = GeoRef->GetTangentTransformAtGeographicLocation(GeoPos);	// ECEFToEngine();
		FVector TanPos = Transform.GetLocation();
		SetActorLocation(TanPos);

		UE_LOG(LogTemp, Warning, TEXT("TanPos: LLA: %lf %lf %lf:   - %lf %lf %lf"), GeoPos.Latitude, GeoPos.Longitude, GeoPos.Altitude,
			TanPos.X, TanPos.Y, TanPos.Z);

		auto OrgPos = FGeographicCoordinates(GeoRef->OriginLongitude, GeoRef->OriginLatitude, GeoRef->OriginAltitude);
		FTransform Orgin = GeoRef->GetTangentTransformAtGeographicLocation(OrgPos);	// ECEFToEngine();
		FVector OrgLoc = Orgin.GetLocation();

		UE_LOG(LogTemp, Warning, TEXT("OrgCoord: LLA: %lf %lf %lf:   - %lf %lf %lf"), GeoRef->OriginLatitude, GeoRef->OriginLongitude, GeoRef->OriginAltitude,
			OrgLoc.X, OrgLoc.Y, OrgLoc.Z);

		double s = 100;
		SetActorScale3D({ s, s, s });


		double ProjMaxRadius = GeoRef->GetProjectedEllipsoidMaxRadius();
		double ProjMinRadius = GeoRef->GetProjectedEllipsoidMinRadius();

		FEllipsoid ProjEllipsoid = FEllipsoid({ ProjMaxRadius, ProjMaxRadius, ProjMinRadius });

		double GeogMaxRadius = GeoRef->GetGeographicEllipsoidMaxRadius();
		double GeogMinRadius = GeoRef->GetGeographicEllipsoidMinRadius();
		FEllipsoid GeogEllipsoid = FEllipsoid({ GeogMaxRadius, GeogMaxRadius, GeogMinRadius });

		UE_LOG(LogTemp, Warning, TEXT("Ellip: Proj: %lf %lf   - Geog: %lf %lf"), ProjMaxRadius, ProjMinRadius, GeogMaxRadius, GeogMinRadius);

	
		// ECEF -> Enu
		FVector Ecef, Enu;
		LLA_TO_ECEF(GeoPos.Latitude, GeoPos.Longitude, GeoPos.Altitude, &Ecef.X, &Ecef.Y, &Ecef.Z);
		ECEF_TO_ENU(GeoRef->OriginLatitude, GeoRef->OriginLongitude, Ecef.X, Ecef.Y, Ecef.Z, 0.0, 0.0, 0.0, &Enu.X, &Enu.Y, &Enu.Z);

		UE_LOG(LogTemp, Warning, TEXT("ECEF-ENU: %lf %lf %lf"), Enu.X, Enu.Y, Enu.Z);
	}
}
