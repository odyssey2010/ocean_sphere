// Fill out your copyright notice in the Description page of Project Settings.


#include "BtOceanSphereActor.h"
#include "GeoReferencingSystem.h"
#include "Kismet/GameplayStatics.h"

#if WITH_EDITOR
//#include "Engine/Selection.h"
//#include "Modules/ModuleManager.h"
//#include "Engine/AssetManager.h"
#include "EditorViewportClient.h"
#endif //WITH_EDITOR

#ifdef WINDOWS
#include <windows.h>
#endif //WINDOWS

// Sets default values
ABtOceanSphereActor::ABtOceanSphereActor()
{
     // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;

    RootComponent = CreateDefaultSubobject<USceneComponent>("Root");

    OceanMesh = CreateDefaultSubobject<UProceduralMeshComponent>("OceanMesh");
    OceanMesh->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
    OceanMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);


    //static FString Path = TEXT("/Game/M_Water");
    //static FString Path = TEXT("/Game/BtC4i3D/Materials/WaterMaterials/Materials/M_Water_Opaque");
    
    //static FString Path = TEXT("/Game/M_Reflection_Test");
    //static FString Path = TEXT("/Game/M_Water");
    static FString Path = TEXT("/Game/M_EarthMap");

    //static FString Path = TEXT("/DataSmithContent/Materials/Water/M_Water");
    //static FString Path = TEXT("/Engine/EngineDebugMaterials/WireframeMaterial");

    static ConstructorHelpers::FObjectFinder<UMaterial> MaterialLoader(*Path);
    if (! MaterialLoader.Succeeded()) {
        UE_LOG(LogTemp, Error, TEXT("Failed to load material at path: %s"), *Path);
    }

    EarthMaterial = MaterialLoader.Object;
    OceanMesh->SetMaterial(0, EarthMaterial);


    static FString Path2 = TEXT("/Game/M_Colored");
    static ConstructorHelpers::FObjectFinder<UMaterial> MaterialLoader2(*Path2);
    if (! MaterialLoader2.Succeeded()) {
        UE_LOG(LogTemp, Error, TEXT("Failed to load material at path: %s"), *Path2);
    }
    ColorMaterial = MaterialLoader2.Object;


    static FString PathTexture = TEXT("/Game/8081_earthspec2k.8081_earthspec2k");
    static ConstructorHelpers::FObjectFinder<UTexture2D> TextureLoader(*PathTexture);
    if (!TextureLoader.Succeeded()) {
        UE_LOG(LogTemp, Error, TEXT("Failed to load height texture at path: %s"), *PathTexture);
    }
    HeightTexture = TextureLoader.Object;
}

void ABtOceanSphereActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    if (GeoRef == nullptr) {
        GeoRef = AGeoReferencingSystem::GetGeoReferencingSystem(GetWorld());

        LoadHeightTexture();

        build();

        build_test();

        update_origin(0);
    }
}

// Called when the game starts or when spawned
void ABtOceanSphereActor::BeginPlay()
{
    Super::BeginPlay();
}

// Called every frame
void ABtOceanSphereActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (GeoRef == nullptr) {
        GeoRef = AGeoReferencingSystem::GetGeoReferencingSystem(GetWorld());

        LoadHeightTexture();

        build();

        build_test();

        update_origin(0);
    }

    //is_check_geo_coord = true;
    //check_geo_coords();

    update_view(0);
}

void ABtOceanSphereActor::PostLoad()
{
    Super::PostLoad();
}

void ABtOceanSphereActor::TestUpdate()
{
    update_view(0);
}

void ABtOceanSphereActor::LoadHeightTexture()
{
    if (HeightTexture) {
        /*
            Mip Gen Settings : NoMipmaps
            sRGB : false
            Compression Settings : TC Vector Displacementmap
        */

        auto& Mips = HeightTexture->GetPlatformMips();
    
        if (Mips.Num() > 0) {
            auto& MipData = (FTexture2DMipMap&) Mips[0];
            ElevDataSize = { MipData.SizeX, MipData.SizeY };
            ElevData.SetNumUninitialized(MipData.SizeX * MipData.SizeY);

            TArray<FColor> PixelData;
            PixelData.SetNumUninitialized(MipData.SizeX * MipData.SizeY);

            const FColor* Pixels = static_cast<const FColor*>(MipData.BulkData.LockReadOnly());

            for (uint32 v = 0; v < MipData.SizeY; ++v) {
                for (uint32 h = 0; h < MipData.SizeX; ++h) {
                    auto& Pixel = Pixels[v * MipData.SizeX + h];
                    ElevData[v * MipData.SizeX + h] = 255 - Pixel.R;
                }
            }

            MipData.BulkData.Unlock();
        }
    }
}

#if WITH_EDITOR
bool ABtOceanSphereActor::ShouldTickIfViewportsOnly() const
{
    return true;
}

void ABtOceanSphereActor::PostEditChangeProperty(FPropertyChangedEvent& e)
{
    ensure(IsValid(this));
    Super::PostEditChangeProperty(e);
    ensure(IsValid(this));
    
    FName PropertyName = (e.Property != NULL) ? e.Property->GetFName() : NAME_None;

    update_view(0);
}
#endif

inline void ECEF_TO_UNREAL(FVector& v) 
{
    v.X *= 100.0;
    v.Y *= -100.0;
    v.Z *= 100.0;
}

inline void UNREAL_TO_ECEF(FVector& v) 
{
    v.X *= 0.01;
    v.Y *= -0.01;
    v.Z *= 0.01;
}

// Converts latitude(deg), longitude(deg) and altitude(m) to ECEF frame
// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates

// WGS84 ellipsoid constants
double WGS84_A = 6378137; // radius
double WGS84_E = 8.1819190842622e-2;  // eccentricity
double DEGREES_TO_RADIANS = PI / 180.0;
double RADIANS_TO_DEGREES = 180.0 / PI;

void LLA_TO_ECEF(double lat, double lon, double alt, double* x, double* y, double* z)
{
    double clat = cos(lat * DEGREES_TO_RADIANS);
    double slat = sin(lat * DEGREES_TO_RADIANS);
    double clon = cos(lon * DEGREES_TO_RADIANS);
    double slon = sin(lon * DEGREES_TO_RADIANS);

    double N = WGS84_A / sqrt(1.0-WGS84_E * WGS84_E * slat * slat);

    *x = (N+alt) * clat * clon;
    *y = (N+alt) * clat * slon;
    *z = (N * (1.0-WGS84_E * WGS84_E)+alt) * slat;
}

// Converts ECEF to ENU coordinates centered at given lat, lon
void ECEF_TO_ENU(double lat, double lon, double x, double y, double z, double xr, double yr, double zr, double* e, double* n, double* u)
{
    double clat = cos(lat * DEGREES_TO_RADIANS);
    double slat = sin(lat * DEGREES_TO_RADIANS);
    double clon = cos(lon * DEGREES_TO_RADIANS);
    double slon = sin(lon * DEGREES_TO_RADIANS);
    double dx = x-xr;
    double dy = y-yr;
    double dz = z-zr;

    *e = -slon * dx+clon * dy;
    *n = -slat * clon * dx-slat * slon * dy+clat * dz;
    *u = clat * clon * dx+clat * slon * dy+slat * dz;
}

void ECEF_TO_LLA(double x, double y, double z, double* _lat, double* _lon, double* _alt) 
{
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
    // https://www.mathworks.com/help/aeroblks/ecefpositiontolla.html

    static const double RADIUS_EQUATOR = 6378137.0;
    static const double FLATTENING_DENOM = 298.257223563;
    static const double FLATTENING = 1.0 / FLATTENING_DENOM;
    static const double RADIUS_POLES = RADIUS_EQUATOR * (1 - FLATTENING);

    static const double a = RADIUS_EQUATOR;
    static const double asqr = a * a;

    static const double b = RADIUS_POLES;
    static const double bsqr = b * b;

    static const double e = sqrt((asqr - bsqr) / asqr);
    static const double eprime = sqrt((asqr - bsqr) / bsqr);

    static const double k1F = FLATTENING_DENOM;
    static const double kE2 = FLATTENING * (2.0 - FLATTENING);


    //Auxiliary values first
    double p = sqrt(x * x + y * y);
    double theta = atan((z * a) / (p * b));

    double sintheta = sin(theta);
    double costheta = cos(theta);

    double num = z + eprime * eprime * b * sintheta * sintheta * sintheta;
    double denom = p - e * e * a * costheta * costheta * costheta;

    //Now calculate LLA
    double lat = atan2(num, denom);
    double lon = atan2(y, x);
    double sinlat = sin(lat);
    double N = a / sqrt(1.0 - kE2 * sinlat * sinlat);
    double alt = (p / cos(lat)) - N;

    *_lat = lat * RADIANS_TO_DEGREES;
    *_lon = lon * RADIANS_TO_DEGREES;
    *_alt = alt;
}

struct SphereBuilder {

    static const int32      VERT_STEP = 18;
    static const int32      HORZ_STEP = 36;
    static const int32      TILE_STEP = 6;


    struct SphereSection;

    void Build_Tiles()
    {
         Sections.SetNum(VERT_STEP * HORZ_STEP);
         Tiles.SetNum(VERT_STEP * HORZ_STEP * TILE_STEP * TILE_STEP);

         struct GeoCoord {
             double Lat;
             double Lon;
         };

        TArray<GeoCoord> GeoCoords;
        GeoCoords.SetNum((VERT_STEP+1) * (HORZ_STEP+1));

        #define GET_INDEX(v, h) ((v) * (HORZ_STEP+1)+(h))

        for (int32 v = 0; v <= VERT_STEP; ++v) {    // [90, -90]
            double AngleVert = 90.0 - (v * 180.0) / VERT_STEP;

            for (int32 h = 0; h <= HORZ_STEP; ++h) {    // [-180, 180]
                double AngleHorz = -180.0 + (h * 360.0) / HORZ_STEP;

                GeoCoords[GET_INDEX(v, h)] = {AngleVert, AngleHorz};
            }
        }

        int32 SectionIndex = 0;
        for (int32 v = 0; v < VERT_STEP; ++v) {
            for (int32 h = 0; h < HORZ_STEP; ++h) {
                auto& LeftTop = GeoCoords[GET_INDEX(v, h)];
                auto& RightBottom = GeoCoords[GET_INDEX(v+1, h+1)];

                SphereSection& Section = Sections[v * HORZ_STEP+h];
                Section.VertIndex = v;
                Section.HorzIndex = h;
                Section.SectionIndex = SectionIndex++;

                Section.Build(RightBottom.Lat, LeftTop.Lat, LeftTop.Lon, RightBottom.Lon, *this);
            }
        }

        static const int32 VERT_STRIDE = VERT_STEP * TILE_STEP;
        static const int32 HORZ_STRIDE = HORZ_STEP * TILE_STEP;

        MinLatitudeDiff = 180.0 / VERT_STRIDE;
        MinLongitudeDiff = 360.0 / HORZ_STRIDE;
        MinAltitude = 1000; // meter

        for (int32 v = 0; v < VERT_STRIDE; ++v) {
            for (int32 h = 0; h < HORZ_STRIDE; ++h) {
                int32 TileIndex = v * HORZ_STRIDE + h;
                auto& Tile = Tiles[TileIndex];

                int32 v0 = (v-1) * HORZ_STRIDE + h;
                int32 v1 = (v+1) * HORZ_STRIDE + h;
                int32 h0 = v * HORZ_STRIDE + (h-1);
                int32 h1 = v * HORZ_STRIDE + (h+1);

                v0 = (v > 0) ? v0 : v * HORZ_STRIDE + h;
                v1 = (v < VERT_STRIDE-1) ? v1 : v * HORZ_STRIDE + h;
                
                h0 = (h > 0) ? h0 : v * HORZ_STRIDE + (HORZ_STRIDE-1);
                h1 = (h < HORZ_STRIDE-1) ? h1 : v * HORZ_STRIDE + 0;
                
                Tile.SideTiles[SphereBuilder::SphereTile::BOTTOM_SIDE] = &Tiles[v0];
                Tile.SideTiles[SphereBuilder::SphereTile::TOP_SIDE] = &Tiles[v1];
                Tile.SideTiles[SphereBuilder::SphereTile::LEFT_SIDE] = &Tiles[h0];
                Tile.SideTiles[SphereBuilder::SphereTile::RIGHT_SIDE] = &Tiles[h1];
            }
        }
    }

    void Elev_Tiles(uint32 Width, uint32 Height, uint8* ElevData)
    {
        static const uint32 VERT_STRIDE = VERT_STEP * TILE_STEP;
        static const uint32 HORZ_STRIDE = HORZ_STEP * TILE_STEP;

        for (uint32 v = 0; v < VERT_STRIDE; ++v) {
            for (uint32 h = 0; h < HORZ_STRIDE; ++h) {
                uint32 TileIndex = v * HORZ_STRIDE + h;
                auto& Tile = Tiles[TileIndex];

                double th = (Tile.LonMin + Tile.LonMax) * 0.5;
                double tv = (Tile.LatMin + Tile.LatMax) * 0.5;
                th = fmod(th + 180.0, 360.0) / 360.0;
                tv = fmod(-tv + 90.0, 180.0) / 180.0;

                uint32 ElevIndexHorz = th * Width;
                uint32 ElevIndexVert = tv * Height;
                uint8 ElevValue = ElevData[ElevIndexVert * Width + ElevIndexHorz];
                Tile.IsOcean = (ElevValue < 5);
            }
        }
    }

    double NormalizeLongitude(double heading) {
        return fmod((heading + 180.0), 360.0) - 180.0;    // Normalize heading to [-180, 180]
    }

    double NormalizeHeading(double heading) {           // Normalize to the range [0, 360)
        heading = fmod(heading, 360.0);
        if (heading < 0) {
            heading += 360.0;
        }
        return heading;
    }

    double NormalizePitch(double pitch) {
        // Step 1: Normalize to the range [-180, 180]
        pitch = fmod(pitch + 180.0, 360.0);
        if (pitch < 0) {
            pitch += 360.0;
        }
        pitch -= 180.0;

        // Step 2: Normalize to the range [-90, 90]
        if (pitch > 90.0) {
            pitch = 180.0 - pitch;
        }
        else if (pitch < -90.0) {
            pitch = -180.0 - pitch;
        }

        return pitch;
    }

    void UpdateTileLevels(double ViewLat, double ViewLon, double ViewAlt) 
    {
        ViewLat = NormalizePitch(ViewLat);
        ViewLon = NormalizeHeading(ViewLon);
        
        const int32 VERT_STRIDE = VERT_STEP * TILE_STEP;
        const int32 HORZ_STRIDE = HORZ_STEP * TILE_STEP;
        
        if (fabs(ViewLatitude - ViewLat) < MinLatitudeDiff &&
            fabs(ViewLongitude - ViewLon) < MinLongitudeDiff &&
            fabs(ViewAltitude - ViewAlt) < MinLongitudeDiff)
            return;

        ViewLatitude = ViewLat;
        ViewLongitude = ViewLon;
        ViewAltitude = ViewAlt;

        //UE_LOG(LogTemp, Log, TEXT("ViewLLA: %f %f %f"), ViewLat, ViewLon, ViewAlt);

        const double alphaH = (ViewLongitude + 180.0) / 360.0;
        const double alphaV = (90.0 - ViewLatitude) / 180.0;

        const int32 viewH = alphaH * HORZ_STRIDE;
        const int32 viewV = alphaV * VERT_STRIDE;

        int32 offset = 5;
        int32 MaxLevel = 5;

        // Clear previous levels
        for (int32 s = 0; s < Sections.Num(); ++s) {
            auto& Section = Sections[s];
            for (int32 t = 0; t < Section.TilePtrs.Num(); ++t) {
                auto& Tile = *Section.TilePtrs[t];

                Tile.Level = 1;
                Section.NeedUpdate = true;
            }
        }

        for (int32 v = viewV - offset; v < viewV + offset; ++v) {
            for (int32 h = viewH - offset; h < viewH + offset; ++h) {
                int32 vi = (v >= 0) ? v % VERT_STRIDE : VERT_STRIDE + v;
                int32 hi = (h >= 0) ? h % HORZ_STRIDE : HORZ_STRIDE + h;

                int32 TileIndex = vi * HORZ_STRIDE + hi;
                auto& Tile = Tiles[TileIndex];

                int32 dv = (v - viewV);
                int32 dh = (h - viewH);
                int32 Distance = sqrt(dv*dv + dh*dh);
                int32 Square = (MaxLevel - Distance);
                if (Square < 1) Square = 1;

                Tile.Level = pow(2, Square);
                Tile.Section->NeedUpdate = true;
            }
        }
    }

    struct SectionMesh {
        TArray<FVector>     Vertices;
        TArray<int32>       Triangles;
        TArray<FVector>     Normals;
        TArray<FVector2D>   TexCoords;
    };

    struct SphereTile {
        double LatMin{ 0.0 }, LatMax{ 0.0 };
        double LonMin{ 0.0 }, LonMax{ 0.0 };
                
        int32 Level{ 1 };
        bool IsOcean{ false };
        SphereSection* Section{ nullptr };

        enum SideEnum {
            TOP_SIDE = 0,
            BOTTOM_SIDE = 1,
            LEFT_SIDE = 2,
            RIGHT_SIDE = 3,
        };
        SphereTile* SideTiles[4] = { nullptr, nullptr, nullptr, nullptr };

        void Build(double latMin, double latMax, double lonMin, double lonMax)
        {
             LatMin = latMin;
             LatMax = latMax;
             LonMin = lonMin;
             LonMax = lonMax;
        }

        void GenerateMesh(SectionMesh& Mesh, SphereBuilder& SB) 
        {
            FVector Ecef;
            FVector Normal;
            FVector2D TexCoord;
            double AlphaV, AlphaH;
            double AngleVert, AngleHorz;
            const double dLevel = (double)Level;
            
            int32 Index = Mesh.Vertices.Num();

            for (int32 v = 0; v <= Level; ++v) {
                AlphaV = v / dLevel;
                AngleVert = LatMax * (1-AlphaV) + LatMin * AlphaV;

                for (int32 h = 0; h <= Level; ++h) {
                    AlphaH = h / dLevel;
                    AngleHorz = LonMin * (1-AlphaH) + LonMax * AlphaH;

                    LLA_TO_ECEF(AngleVert, AngleHorz, SB.OceanHeight, &Ecef.X, &Ecef.Y, &Ecef.Z);
                    ECEF_TO_UNREAL(Ecef);

                    Normal = Ecef;
                    Normal.Normalize();

                    TexCoord.X = 0.5 + AngleHorz / 360.0;
                    TexCoord.Y = 0.5 - AngleVert / 180.0;

                    Mesh.Vertices.Add(Ecef);
                    Mesh.Normals.Add(Normal);
                    Mesh.TexCoords.Add(TexCoord);
                }
            }

            #define TILE_INDEX(vert, horz) ((vert) * (Level+1) + (horz))

            for (int32 v = 0; v < Level; ++v) {
                for (int32 h = 0; h < Level; ++h) {
                    int32 i0 = TILE_INDEX(v, h), i1 = i0+1;
                    int32 i2 = TILE_INDEX(v+1, h), i3 = i2+1;

                    Mesh.Triangles.Append({ Index+i0, Index+i1, Index+i2, Index+i2, Index+i1, Index+i3 });
                }
            }

            if (Level > 1) {
                FVector* Coords = &Mesh.Vertices[Index];
                for (int32 i = 0; i < 4; ++i) {
                    auto SideTile = SideTiles[i];
                    if (SideTile->Level < Level) {
                        StitchSideVertices((SideEnum) i, Coords, SideTile->Level);
                    }
                }
            }
        }

        void StitchSideVertices(SideEnum Side, FVector* Coords, int32 DstLevel) 
        {
            static TArray<FVector*> Vertices;
            Vertices.SetNum(Level+1, false);

            switch (Side) {
            case TOP_SIDE:      for (int32 h = 0; h <= Level; ++h) Vertices[h] = &Coords[TILE_INDEX(0, h)]; break;
            case BOTTOM_SIDE:   for (int32 h = 0; h <= Level; ++h) Vertices[h] = &Coords[TILE_INDEX(Level, h)]; break;
            case LEFT_SIDE:     for (int32 v = 0; v <= Level; ++v) Vertices[v] = &Coords[TILE_INDEX(v, 0)]; break;
            case RIGHT_SIDE:    for (int32 v = 0; v <= Level; ++v) Vertices[v] = &Coords[TILE_INDEX(v, Level)]; break;
            }

            if (Level / DstLevel > 2) {
                double LogSrc = log2(Level);
                double LogDst = log2(DstLevel);
                // TODO:
                UE_LOG(LogTemp, Log, TEXT("Stitch %d -> %d"), Level, DstLevel);
            }

            for (int32 i = 1; i < Level; i += 2) {
                FVector& V0 = *Vertices[i-1];
                FVector& V1 = *Vertices[i];
                FVector& V2 = *Vertices[i+1];

                V1 = (V0 + V2) * 0.5;
            }
        }
    };

    struct SphereSection {
        void Build(double latMin, double latMax, double lonMin, double lonMax, SphereBuilder& SB)
        {
            LatMin = latMin;
            LatMax = latMax;
            LonMin = lonMin;
            LonMax = lonMax;
              
            TilePtrs.SetNum(TILE_STEP * TILE_STEP);

            const int32 HORZ_STRIDE = SphereBuilder::HORZ_STEP * TILE_STEP;
            const int32 TILE_START = (VertIndex * HORZ_STRIDE + HorzIndex) * TILE_STEP;
            const double dTILE_STEP = (double)TILE_STEP;

            for (int32 v = 0; v < TILE_STEP; ++v) {
                for (int32 h = 0; h < TILE_STEP; ++h) {
                    int32 TileIndex = TILE_START + (v * HORZ_STRIDE + h);

                    SphereTile& Tile = SB.Tiles[TileIndex];
                    Tile.Section = this;
                    TilePtrs[v * TILE_STEP + h] = &Tile;

                    double alphaV0 = v / dTILE_STEP;
                    double alphaV1 = (v+1) / dTILE_STEP;
                    double alphaH0 = h / dTILE_STEP;
                    double alphaH1 = (h+1) / dTILE_STEP;

                    double V0 = LatMax * (1-alphaV0) + LatMin * alphaV0;
                    double V1 = LatMax * (1-alphaV1) + LatMin * alphaV1;
                    double H0 = LonMin * (1-alphaH0) + LonMax * alphaH0;
                    double H1 = LonMin * (1-alphaH1) + LonMax * alphaH1;

                    Tile.Build(V0, V1, H0, H1);
                }
            }
        }

        void GenerateMesh(SphereBuilder& SB) 
        {
            if (!NeedUpdate)
                return;

            auto& Mesh = SB.BufferMesh ;
            Mesh.Vertices.SetNumUninitialized(0, false);
            Mesh.Triangles.SetNumUninitialized(0, false);
            Mesh.Normals.SetNumUninitialized(0, false);
            Mesh.TexCoords.SetNumUninitialized(0, false);

            for (auto Tile : TilePtrs) {
                //if (Tile->IsOcean)
                    Tile->GenerateMesh(Mesh, SB);
            }

            SB.OceanMesh->CreateMeshSection(SectionIndex, Mesh.Vertices, Mesh.Triangles, Mesh.Normals, Mesh.TexCoords, {}, {}, false);
            SB.OceanMesh->SetMaterial(SectionIndex, SB.EarthMaterial);

            NeedUpdate = false;
        }

        double LatMin{ 0.0 }, LatMax{ 0.0 };
        double LonMin{ 0.0 }, LonMax{ 0.0 };

        int32 VertIndex{ 0 };
        int32 HorzIndex{ 0 };
        int32 SectionIndex{ 0 };
        bool NeedUpdate{ true };

        TArray<SphereTile*> TilePtrs;
    };

public:
    double                  ViewLatitude{ 0.0 };
    double                  ViewLongitude{ 0.0 };
    double                  ViewAltitude{ 0.0 };

    double                  MinLatitudeDiff{ 0.0001 };
    double                  MinLongitudeDiff{ 0.0001 };
    double                  MinAltitude{ 1000 };

    double                  OceanHeight{ 0 };

    UProceduralMeshComponent* OceanMesh{ nullptr };
    UMaterial*              EarthMaterial{ nullptr };
    TArray<SphereSection>   Sections;
    TArray<SphereTile>      Tiles;

    SectionMesh             BufferMesh ;
};

void ABtOceanSphereActor::build()
{
    SB = new SphereBuilder();
    SB->OceanHeight = OceanHeight;
    SB->OceanMesh = OceanMesh;
    SB->EarthMaterial = EarthMaterial;

    SB->Build_Tiles();

    SB->Elev_Tiles(ElevDataSize.Width, ElevDataSize.Height, ElevData.GetData());

    SB->UpdateTileLevels(TestLatitude, TestLongitude, 0);

    for (auto& Section : SB->Sections) {
        Section.GenerateMesh(*SB);
    }

    TestSectionIndex = SB->Sections.Num();
}

void ABtOceanSphereActor::build_test()
{
    int32 sectionIndex = TestSectionIndex;
    TArray<FVector> Vertices;
    TArray<FVector> Normals;
    TArray<FColor> Colors;
    TArray<FVector2D> UV0;
    TArray<int32> Triangles;

    struct {
        double Lat;
        double Lon;
        double Alt;
    } GeoPoints[] = {
    { 24.747457, 52.069997, 0.0 },
    { 23.881029, 51.976362, 0.0 },
    };
    const double ALT_OFFSET = 100000;

    const int32 PATH_STEP = 200;
    const auto P0 = GeoPoints[0];
    const auto P1 = GeoPoints[1];

    FVector Pv0, Pv1, Cv0, Cv1;
    for (int32 i = 0; i <= PATH_STEP; ++i) {
        double t = i / (double)PATH_STEP;
        double lat_ip = P0.Lat * (1-t) + P1.Lat * t;
        double lon_ip = P0.Lon * (1-t) + P1.Lon * t;
        double alt_ip = P0.Alt * (1-t) + P1.Alt * t;

        LLA_TO_ECEF(lat_ip, lon_ip, 0.0, &Cv0.X, &Cv0.Y, &Cv0.Z);
        ECEF_TO_UNREAL(Cv0);

        LLA_TO_ECEF(lat_ip, lon_ip, ALT_OFFSET, &Cv1.X, &Cv1.Y, &Cv1.Z);
        ECEF_TO_UNREAL(Cv1);

        Vertices.Append({ Cv0, Cv1 });
        Colors.Append({ FColor::Green, FColor::Blue });

        if (i > 0) {
            int32 i0 = (i-1) * 2, i1 = i0+1;
            int32 i2 = i * 2, i3 = i2+1;
            Triangles.Append({ i0, i1, i2, i2, i1, i3 });
        }

        Pv0 = Cv0;
        Pv1 = Cv1;
    }

    OceanMesh->CreateMeshSection(sectionIndex, Vertices, Triangles, {}, {}, Colors, {}, false);
    OceanMesh->SetMaterial(sectionIndex, ColorMaterial);


    FVector PrevG, CurrG;
    for (int32 i = 0; i <= PATH_STEP; ++i) {
        double t = i / (double)PATH_STEP;
        double lat_ip = P0.Lat * (1-t) + P1.Lat * t;
        double lon_ip = P0.Lon * (1-t) + P1.Lon * t;
        double alt_ip = P0.Alt * (1-t) + P1.Alt * t;

        FGeographicCoordinates G{ lon_ip, lat_ip, alt_ip };
        CurrG = GeoRef->GetTangentTransformAtGeographicLocation(G).GetLocation();

        if (i > 0) {
            DrawDebugLine(GetWorld(), PrevG, CurrG, FColor::Red, true, 10000.0f, 0U, 100000);
        }

        PrevG = CurrG;
    }
}

void ABtOceanSphereActor::build_test_view(double ViewLat, double ViewLon, double ViewAlt)
{
    int32 sectionIndex = TestSectionIndex + 1;
    TArray<FVector> Vertices;
    TArray<FVector> Normals;
    TArray<FColor> Colors;
    TArray<FVector2D> UV0;
    TArray<int32> Triangles;

    const double ALT_OFFSET = 200000;

    FVector2D Offsets[5] = {
        { -0.01, -0.01 },
        {  0.01, -0.01 },
        {  0.01,  0.01 },
        { -0.01,  0.01 },
        { -0.01, -0.01 }
    };

    FVector Pv0, Pv1, Cv0, Cv1;
    for (int32 i = 0; i < 5; ++i) {
        FVector2D& Offset = Offsets[i];

        LLA_TO_ECEF(ViewLat + Offset.X, ViewLon + Offset.Y, 0.0, &Cv0.X, &Cv0.Y, &Cv0.Z);
        ECEF_TO_UNREAL(Cv0);

        LLA_TO_ECEF(ViewLat + Offset.X, ViewLon + Offset.Y, ALT_OFFSET, &Cv1.X, &Cv1.Y, &Cv1.Z);
        ECEF_TO_UNREAL(Cv1);

        int vindex = Vertices.Num();
        Vertices.Append({ Cv0, Cv1 });
        Colors.Append({ FColor::Green, FColor::Blue });

        if (i > 0) {
            int32 i0 = (i-1)*2, i1 = i0 + 1;
            int32 i2 = i*2, i3 = i2 + 1;
            Triangles.Append({ i0, i1, i2, i2, i1, i3 });
        }

        Pv0 = Cv0;
        Pv1 = Cv1;
    }

    OceanMesh->CreateMeshSection(sectionIndex, Vertices, Triangles, {}, {}, Colors, {}, false);
    OceanMesh->SetMaterial(sectionIndex, ColorMaterial);
}

struct GeographicImpl {
    FEllipsoid ProjectedEllipsoid{ 6378137.0000000000, 6378137.0000000000, 6356752.3142451793 };
    FEllipsoid GeographicEllipsoid{ 6378137.0000000000, 6378137.0000000000, 6356752.3142451793 };

    FMatrix WorldFrameToECEFFrame = {
    {-0.80901699437494745, 0.58778525229247314, 0.0000000000000000, 0.0000000000000000},
    {-0.23907380036690279, -0.32905685648333960, 0.91354545764260098, 0.0000000000000000},
    {0.53696854730109900, 0.73907380036690284, 0.40673664307580015, 0.0000000000000000},
    {3426757.0221417057, 4716526.4111234806, 2578283.4220752530, 1.0000000000000000}
    };
    FMatrix ECEFFrameToWorldFrame = {
    {-0.80901699437494745, -0.23907380036690279, 0.53696854730109900, -0.0000000000000000},
    {0.58778525229247314, -0.32905685648333960, 0.73907380036690296, 0.0000000000000000},
    {0.0000000000000000, 0.91354545764260098, 0.40673664307580015, -0.0000000000000000},
    {4.6566128730773926e-10, 15874.069830260240, -6374604.1833260469, 1.0000000000000000}
    };
    FMatrix WorldFrameToUEFrame = {
    {1.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000},
    {0.0000000000000000, -1.0000000000000000, 0.0000000000000000, 0.0000000000000000},
    {0.0000000000000000, 0.0000000000000000, 1.0000000000000000, 0.0000000000000000},
    {0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000}
    };
    FMatrix UEFrameToWorldFrame = {
    {1.0000000000000000, 0.0000000000000000, -0.0000000000000000, 0.0000000000000000},
    {0.0000000000000000, -1.0000000000000000, 0.0000000000000000, -0.0000000000000000},
    {-0.0000000000000000, 0.0000000000000000, 1.0000000000000000, 0.0000000000000000},
    {0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000}
    };

    // LWC_TODO-To be replaced once FVector::Normalize will use a smaller number than 1e-8
#define GEOREF_DOUBLE_SMALL_NUMBER (1.e-50)

    FTransform GetTangentTransformAtECEFLocation(const FVector& ECEFCoordinates)
    {
        FMatrix WorldFrameToECEFFrameAtLocation = GetWorldFrameToECEFFrame(GeographicEllipsoid, ECEFCoordinates);

        FMatrix UEtoECEF = UEFrameToWorldFrame * WorldFrameToECEFFrameAtLocation * ECEFFrameToWorldFrame * WorldFrameToUEFrame;
        FVector UEOrigin;
        ECEFToEngine(ECEFCoordinates, UEOrigin);
        UEtoECEF.SetOrigin(UEOrigin);

        return FTransform(UEtoECEF);
    }

    FMatrix GetWorldFrameToECEFFrame(const FEllipsoid& Ellipsoid, const FVector& ECEFLocation)
    {
        // See ECEF standard : https://commons.wikimedia.org/wiki/File:ECEF_ENU_Longitude_Latitude_right-hand-rule.svg

        FVector Up = Ellipsoid.GeodeticSurfaceNormal(ECEFLocation);
        FVector East(-ECEFLocation.Y, ECEFLocation.X, 0.0);
        East.Normalize(GEOREF_DOUBLE_SMALL_NUMBER);
        FVector North = Up.Cross(East);
        return FMatrix(East, North, Up, ECEFLocation);
    }

    void ECEFToEngine(const FVector& ECEFCoordinates, FVector& EngineCoordinates)
    {
        FVector UEWorldCoordinates;

        UEWorldCoordinates = ECEFFrameToWorldFrame.TransformPosition(ECEFCoordinates);

        // Convert UE units to meters, invert the Y coordinate because of left-handed UE Frame
        EngineCoordinates = UEWorldCoordinates * FVector(100.0, -100.0, 100.0);
    }

};

void ABtOceanSphereActor::update_origin(float DeltaTime)
{
    double Latitude = GeoRef->OriginLatitude;
    double Longitude = GeoRef->OriginLongitude;
    double Altitude = GeoRef->OriginAltitude;

    if (1) {
        Latitude = TestLatitude;
        Longitude = TestLongitude;
        Altitude = 0.0;
    }

    FVector GeoECEF;    //GeoRef->GeographicToECEF(FGeographicCoordinates(Longitude, Latitude, Altitude), GeoECEF);

    LLA_TO_ECEF(Latitude, Longitude, 0.0, &GeoECEF.X, &GeoECEF.Y, &GeoECEF.Z);
    ECEF_TO_UNREAL(GeoECEF);


    GeographicImpl impl;

    FMatrix M = impl.GetWorldFrameToECEFFrame(impl.GeographicEllipsoid, GeoECEF);
    FMatrix M_inv = M.Inverse();
    FTransform T;
    T.SetFromMatrix(M_inv);

    SetActorLocation({ 0.0, 0.0, 0.0 });
    SetActorRotation({ 0.0, 0.0, 0.0 });
    OceanMesh->SetRelativeTransform(T);

    if (0) {
        double X = 1199816.63380;
        double Y = -4812201.6785;
        double Z = 4016140.62425;
        double ViewLat, ViewLon, ViewAlt;
        ECEF_TO_LLA(X, Y, Z, &ViewLat, &ViewLon, &ViewAlt);

        //Lat : 39.111677542
        //Lon : -76
        //Alt : 12068.6198
         
        UE_LOG(LogTemp, Warning, TEXT("ECEF  %f %f %f to LLA: %f %f %f"), X, Y, Z, ViewLat, ViewLon, ViewAlt);
    }

    if (0) {

        FVector CameraLocation = { -249582.787664, 8056040.100924, -4078010.822198 };

        FVector LocalLocation = GetActorTransform().Inverse().TransformPosition(CameraLocation);

        FVector ViewECEF = M.TransformPosition(LocalLocation);

        UNREAL_TO_ECEF(ViewECEF);
        double ViewLat, ViewLon, ViewAlt;
        ECEF_TO_LLA(ViewECEF.X, ViewECEF.Y, ViewECEF.Z, &ViewLat, &ViewLon, &ViewAlt);

        if (isnan(ViewLat) || isnan(ViewLon))
            return;

        SB->UpdateTileLevels(ViewLat, ViewLon, ViewAlt);
        for (auto& Section : SB->Sections) {
            Section.GenerateMesh(*SB);
        }

    }

    /*
    if (1) {
        //FVector CameraLocation = { -18736629.953639, -7674744.173562, 398533.519322 };

        //FVector CameraLocation = { 64578585.601, 80528982.401, 34184804 };

        //FVector CameraLocation = { 131964.016397, 8047375.066704, 647373.486978 };

        FVector CameraLocation = { -249582.787664, 8056040.100924, - 4078010.822198 };

        FVector LocalLocation = GetActorTransform().Inverse().TransformPosition(CameraLocation);
        //FVector LocalLocation = GetActorTransform().TransformPosition(CameraLocation);

        FVector ViewECEF = M.TransformPosition(LocalLocation);

        UNREAL_TO_ECEF(ViewECEF);
        double ViewLat, ViewLon, ViewAlt;
        ECEF_TO_LLA(ViewECEF.X, ViewECEF.Y, ViewECEF.Z, &ViewLat, &ViewLon, &ViewAlt);

        UE_LOG(LogTemp, Warning, TEXT("ECEF  %f %f %f to LLA: %f %f %f"), ViewECEF.X, ViewECEF.Y, ViewECEF.Z, ViewLat, ViewLon, ViewAlt);

        SB->UpdateTileLevels(ViewLat, ViewLon);
        for (auto& Section : SB->Sections) {
            Section.GenerateMesh(*SB);
        }
    }
    */

    if (1) {
        build_test_view(Latitude, Longitude, 0);
    }
}

void ABtOceanSphereActor::update_view(float DeltaTime)
{
    // https://forums.unrealengine.com/t/accessing-the-position-of-the-editor-viewport-camera/297487/11
    // https://forums.unrealengine.com/t/getting-editor-viewport-camera/275853/3
    
    FVector CameraLocation;

    if (auto PlayerCameraManager = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0)) {
        CameraLocation = PlayerCameraManager->GetCameraLocation();

        if (CameraLocation.Equals(FVector::ZeroVector))
            return;
    }
    else {
#if WITH_EDITOR
        FViewport* activeViewport = GEditor->GetActiveViewport();
        if (!activeViewport)
            return;

        FEditorViewportClient* editorViewClient = (FEditorViewportClient*) (activeViewport->GetClient());
        if (!editorViewClient)
            return;

        FVector viewPos = editorViewClient->GetViewLocation();
        FRotator viewRot = editorViewClient->GetViewRotation();
        
        CameraLocation = viewPos;

        //UE_LOG(LogTemp, Warning, TEXT("Camera %s"), *CameraLocation.ToString());
#endif  //WITH_EDITOR
    }


    double Latitude = GeoRef->OriginLatitude;
    double Longitude = GeoRef->OriginLongitude;
    double Altitude = GeoRef->OriginAltitude;

    if (1) {
        Latitude = TestLatitude;
        Longitude = TestLongitude;
        Altitude = 0.0;
    }

    FVector GeoECEF;    //GeoRef->GeographicToECEF(FGeographicCoordinates(Longitude, Latitude, Altitude), GeoECEF);

    LLA_TO_ECEF(Latitude, Longitude, 0.0, &GeoECEF.X, &GeoECEF.Y, &GeoECEF.Z);
    ECEF_TO_UNREAL(GeoECEF);

    GeographicImpl impl;

    FMatrix M = impl.GetWorldFrameToECEFFrame(impl.GeographicEllipsoid, GeoECEF);
    FMatrix M_inv = M.Inverse();
    FTransform T;
    T.SetFromMatrix(M_inv);

    if (1) {
        FVector LocalLocation = GetActorTransform().Inverse().TransformPosition(CameraLocation);

        FVector ViewECEF = M.TransformPosition(LocalLocation);

        UNREAL_TO_ECEF(ViewECEF);
        double ViewLat, ViewLon, ViewAlt;
        ECEF_TO_LLA(ViewECEF.X, ViewECEF.Y, ViewECEF.Z, &ViewLat, &ViewLon, &ViewAlt);
  
        if (isnan(ViewLat) || isnan(ViewLon))
            return;

        SB->UpdateTileLevels(ViewLat, ViewLon, ViewAlt);
        for (auto& Section : SB->Sections) {
            Section.GenerateMesh(*SB);
        }

#ifdef WINDOWS
        if (GetKeyState(VK_SPACE) & 0x8000) {
            FString LogText = FString::Printf(TEXT("ECEF  %f %f %f to LLA: %f %f %f"), ViewECEF.X, ViewECEF.Y, ViewECEF.Z, ViewLat, ViewLon, ViewAlt);
            UE_LOG(LogTemp, Warning, TEXT("%s"), *LogText);

            //GEngine->AddOnScreenDebugMessage(0, 15.0f, FColor::Red, *LogText);

            build_test_view(ViewLat, ViewLon, ViewAlt);
        }
#endif  //WINDOWS
    }
}

#ifdef WINDOWS
class MyThread : public FRunnable {
public:
    FRunnableThread* Thread{ nullptr };
    bool bStopThread = false;
    int th = 0;
    int TargetCount = 10;
    int FoundCount = 0;
    TArray<int> ProcessedNumbers;

    MyThread(int threadCount)
    {
        th = threadCount;
        TargetCount = 10;
        FoundCount = 0;

        Thread = FRunnableThread::Create(this, TEXT("Thread") + th);
    }

    ~MyThread()
    {
        if (Thread != nullptr) {
            Thread->Kill(true);
            delete Thread;
        }
    }

    virtual bool Init() override {
        FRunnable::Init();
        return false;
    }

    virtual uint32 Run() override
    {
        GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("running thread " + FString::FromInt(th)));
        UE_LOG(LogTemp, Warning, TEXT("running thread %d"), th);

        bStopThread = false;
        while (!bStopThread && FoundCount < TargetCount) {
            if (FoundCount == 2) {
                GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("break"));
                UE_LOG(LogTemp, Warning, TEXT("break"));
                //Stop();

            }

            GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("fc =  " + FString::FromInt(FoundCount)));
            UE_LOG(LogTemp, Warning, TEXT("fc =  %d"), FoundCount);
            int32 x = 0;
            while (x < INT_MAX)
            {
                x++;
            }
            ProcessedNumbers.Add(FMath::RandRange(0, 999));
            FoundCount += 1;
        }
        //OnCompleteDelegate.ExecuteIfBound();
        return 0;
    }
    virtual void Exit() override
    {
        FRunnable::Exit();
        GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("exit"));
        UE_LOG(LogTemp, Warning, TEXT("exit"));

    }
    virtual void Stop() override
    {
        FRunnable::Stop();
        bStopThread = true;
    }
};
#endif  //WINDOWS

#include <cstdio>
#include <fstream>

void ABtOceanSphereActor::check_geo_coords()
{
    if (!is_check_geo_coord)
        return;

    if (GeoRef == nullptr) {
        GeoRef = AGeoReferencingSystem::GetGeoReferencingSystem(GetWorld());
        if (!GeoRef)
            return;
    }

    is_check_geo_coord = false;

    int32 CHECK_STEPS = 36;
    FVector GeoECEF, pos;

    FString filename = FPaths::Combine(FPaths::ProjectDir(), "geo_coords.csv");
    std::ofstream file;
    file.open(*filename, std::ofstream::out);
    
    double longitude = 54;
    for (int32 i = 0; i < CHECK_STEPS; ++i) {
        double AngleVert = -(i / (double)(CHECK_STEPS+1) - 0.5) * 180;
        GeoRef->GeographicToECEF(FGeographicCoordinates{ longitude, AngleVert, 0.0 }, GeoECEF);

        file << AngleVert << ", " << GeoECEF.X << ", " << GeoECEF.Y << ", " << GeoECEF.Z << "\n";

        LLA_TO_ECEF(AngleVert, longitude, 0, &pos.X, &pos.Y, &pos.Z);
        file << AngleVert << ", " << pos.X << ", " << pos.Y << ", " << pos.Z << "\n";
    }

    file.close();
}