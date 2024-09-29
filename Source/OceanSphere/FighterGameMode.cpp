// Copyright Epic Games, Inc. All Rights Reserved.

#include "FighterGameMode.h"
#include "FighterPawn.h"
#include "HelicopterPawn.h"

AFighterGameMode::AFighterGameMode()
{
	// set default pawn class to our flying pawn
	//DefaultPawnClass = AFighterPawn::StaticClass();

	DefaultPawnClass = AHelicopterPawn::StaticClass();

	/*static ConstructorHelpers::FObjectFinder<UClass> PawnClass(TEXT("Blueprint'/Game/BP_HelicopterPawn.BP_HelicopterPawn_C'"));
	if (PawnClass.Object != NULL)
	{
		DefaultPawnClass = PawnClass.Object.Get();
	}*/

	/*
	// Structure to hold one - time initialization
	struct FConstructorStatics
	{
		ConstructorHelpers::FObjectFinderOptional<UClass> PawnClass;
		FConstructorStatics()
			: PawnClass(TEXT("/Game/BP_HelicopterPawn.BP_HelicopterPawn_C"))
		{
		}
	};
	static FConstructorStatics ConstructorStatics;

	DefaultPawnClass = ConstructorStatics.PawnClass.Get();
	*/
}
