(* ::Package:: *)

(*
	Copyright 2016 Tom Dela Haije

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*)

(*Fix extensions export*)
(*Allow separate header and image file in export*)
(*Check utf-8 characterencoding*)
(*Leave out "NIfTIUnused" because they are not used in the standard.*)
(*Add "Extensions" -> Automatic setting*)
(*Add duplicate spatial transform option to "Compatibility"*)

Begin["Diffusica`Convert`NIfTIDump`"];

(*Conversion tables*)
$CompatibilityModes = Dispatch[{
	None 		-> ({"", "", 0, 0, "", 0, 0} &), 
	"Analyze"	-> ({"", "", 16384, 0, "r", Ceiling@Max[#], Floor@Min[#]} &)
}];

$DataTypes = Dispatch[{
	0 		-> Undefined, 
	2 		-> "Byte",
	4 		-> "Integer16", 
	8 		-> "Integer32", 
	16 		-> "Real32", 
	32 		-> "Complex64", 
	64 		-> "Real64",
	128 	-> {"Byte", "Byte", "Byte"},
	256 	-> "Integer8",
	512 	-> "UnsignedInteger16",
	768 	-> "UnsignedInteger32",
	1024 	-> "Integer64",
	1280 	-> "UnsignedInteger64",
	1536 	-> "Real128",
	1792 	-> "Complex128",
	2048 	-> "Complex256"
}];

$Directions = Dispatch[{
	0 	-> Undefined, 
	1 	-> "x", 
	2 	-> "y", 
	3	-> "z"
}];

$ECodes = Dispatch[{
	0		-> "UnknownPrivateFormat",
	2		-> "DICOM", 
	4		-> "AFNI",
	6		-> "Comment",
	8		-> "XCEDE",
	10		-> "JIM",
	12		-> "FWDS",
	code_	:> Undefined[code]
}];

$Forms = Dispatch[{
	0 	-> "ArbitraryCoordinates", 
	1 	-> "ScannerBasedAnatomicalCoordinates",
	2	-> "CoregisteredCoordinates",
	3	-> "TalairachTournouxAtlasAlignedCoordinates",
	4	-> "MNI152NormalizedCoordinates"
}];

$IntentCodes = Dispatch[{
	0 			-> {Undefined &, 0}, 
	2      		-> {"Statistic" &, 0},
	3 			-> {StudentTDistribution, 1},
	4 			-> {FRatioDistribution, 2},
	5 			-> {NormalDistribution, 0},
	6 			-> {ChiSquareDistribution, 1},
	7 			-> {BetaDistribution, 2}, 
	8 			-> {BinomialDistribution, 2},
	9 			-> {GammaDistribution, 2},
	10			-> {PoissonDistribution, 1},
	11			-> {NormalDistribution, 2},
	12			-> {NoncentralFRatioDistribution, 3},
	13			-> {NoncentralChiSquareDistribution, 2},
	14			-> {LogisticDistribution, 2},
	15			-> {LaplaceDistribution, 2},
	16			-> {UniformDistribution[{#1, #2}] &, 2},
	17			-> {NoncentralStudentTDistribution, 2},
	18			-> {WeibullDistribution[#3, #2, #1] &, 3},
	19			-> {ChiDistribution, 1},
	20			-> {InverseGaussianDistribution, 2},
	21			-> {ExtremeValueDistribution, 2},
	22			-> {"PValue" &, 0},
	23			-> {"LogPValue" &, 0},
	24			-> {"Log10PValue" &, 0},
	1001		-> {"Estimate" &, 0},
	1002		-> {"Label" &, 0},
	1003		-> {"Neuroname" &, 0},
	1004		-> {{"Matrix", {#1, #2}} &, 2},
	1005		-> {{"SymmetricMatrix", {#, #}} &, 1},
	1006		-> {"DisplacementVector" &, 0},
	1007		-> {"Vector" &, 0},
	1008		-> {"Pointset" &, 0},
	1009		-> {"Triangle" &, 0},
	1010		-> {"Quaternion" &, 0},
	1011		-> {"Dimensionless" &, 0},
	_Integer	:> {Undefined &, 0}
}];

$RangeChecks = Dispatch[{
	Undefined 					-> Undefined,
	"Byte" 						-> {0, 255},
	"Integer16" 				-> {-32768, 32767},
	"Integer32" 				-> {-2147483648, 2147483647},
	"Real32" 					-> Undefined,
	"Complex64" 				-> Undefined,
	"Real64" 					-> Undefined, 
	{"Byte", "Byte", "Byte"} 	-> {0, 255},
	"Integer8" 					-> {-128, 127},
	"UnsignedInteger16" 		-> {0, 65535},
	"UnsignedInteger32" 		-> {0, 4294967295},
	"Integer64" 				-> {-9223372036854775808, 9223372036854775807},
	"UnsignedInteger64" 		-> {0, 18446744073709551615},
	"Real128" 					-> Undefined,
	"Complex128" 				-> Undefined,
	"Complex256" 				-> Undefined
}];

$ReverseDataTypes = Dispatch[{
	Undefined 					-> 0,
	"Byte" 						-> 2,
	"Integer16" 				-> 4,
	"Integer32" 				-> 8,
	"Real32" 					-> 16,
	"Complex64" 				-> 32,
	"Real64" 					-> 64, 
	{"Byte", "Byte", "Byte"} 	-> 128,
	"Integer8" 					-> 256,
	"UnsignedInteger16" 		-> 512,
	"UnsignedInteger32" 		-> 768,
	"Integer64" 				-> 1024,
	"UnsignedInteger64" 		-> 1280,
	"Real128" 					-> 1536,
	"Complex128" 				-> 1792,
	"Complex256" 				-> 2048
}];

$ReverseDirections = Dispatch[{
	Undefined	-> 0, 
	"x"			-> 1, 
	"y"			-> 2, 
	"z"			-> 3
}];

$ReverseECodes = Dispatch[{
	"UnknownPrivateFormat"				-> 0,
	"DICOM"								-> 2, 
	"AFNI"								-> 4,	
	"Comment"							-> 6,
	"XCEDE"								-> 8,
	"JIM"								-> 10,
	"FWDS"								-> 12,
	Undefined[code_Integer ? Positive]	:> code
}];

$ReverseForms = Dispatch[{
	"ArbitraryCoordinates" 						-> 0, 
 	"ScannerBasedAnatomicalCoordinates" 		-> 1, 
 	"CoregisteredCoordinates" 					-> 2, 
 	"TalairachTournouxAtlasAlignedCoordinates"	-> 3, 
 	"MNI152NormalizedCoordinates" 				-> 4
}];

$ReverseIntentCodes = Dispatch[{
	Undefined									-> {0, {}},
	"Statistic" 								-> {2, {}}, 
	StudentTDistribution[p1_] 					:> {3, {p1}},
	StudentTDistribution	 					-> {3, {0.}},
	FRatioDistribution[p1_, p2_]				:> {4, {p1, p2}},
	FRatioDistribution							-> {4, {0., 0.}},
	NormalDistribution							-> {5, {}}, 
	ChiSquareDistribution[p1_] 					:> {6, {p1}},
	ChiSquareDistribution						-> {6, {0.}},
	BetaDistribution[p1_, p2_]					:> {7, {p1, p2}},
	BetaDistribution							-> {7, {0., 0.}},
	BinomialDistribution[p1_, p2_] 				:> {8, {p1, p2}},
	BinomialDistribution		 				-> {8, {0., 0.}}, 
 	GammaDistribution[p1_, p2_] 				:> {9, {p1, p2}},
 	GammaDistribution		 					-> {9, {0., 0.}}, 
 	PoissonDistribution[p1_]					:> {10, {p1}},
 	PoissonDistribution							-> {10, {0.}},  
 	NormalDistribution[p1_, p2_] 				:> {11, {p1, p2}},
 	NormalDistribution			 				-> {11, {0., 0.}}, 
 	NoncentralFRatioDistribution[p1_, p2_, p3_] :> {12, {p1, p2, p3}},
 	NoncentralFRatioDistribution				-> {12, {0., 0., 0.}}, 
 	NoncentralChiSquareDistribution[p1_, p2_] 	:> {13, {p1, p2}},
 	NoncentralChiSquareDistribution			 	-> {13, {0., 0.}},  
 	LogisticDistribution[p1_, p2_] 				:> {14, {p1, p2}},
 	LogisticDistribution		 				-> {14, {0., 0.}},  
 	LaplaceDistribution[p1_, p2_]				:> {15, {p1, p2}}, 
 	LaplaceDistribution							-> {15, {0., 0.}}, 
 	UniformDistribution[{p1_, p2_}]				:> {16, {p1, p2}},
 	UniformDistribution[{#1, #2}] &				-> {16, {0., 0.}}, 
 	NoncentralStudentTDistribution[p1_, p2_] 	:> {17, {p1, p2}},
 	NoncentralStudentTDistribution			 	-> {17, {0., 0.}}, 
 	WeibullDistribution[p1_, p2_, p3_] 			:> {18, {p3, p2, p1}}, 
 	WeibullDistribution[#3, #2, #1] & 			-> {18, {0., 0., 0.}},
 	ChiDistribution[p1_]						:> {19, {p1}},
 	ChiDistribution								-> {19, {0.}},  
 	InverseGaussianDistribution[p1_, p2_] 		:> {20, {p1, p2}},
 	InverseGaussianDistribution			 		-> {20, {0., 0.}}, 
 	ExtremeValueDistribution[p1_, p2_] 			:> {21, {p1, p2}},
 	ExtremeValueDistribution		 			-> {21, {0., 0.}}, 
 	"PValue" 									-> {22, {}}, 
 	"LogPValue" 								-> {23, {}}, 
 	"Log10PValue" 								-> {24, {}}, 
 	"Estimate" 									-> {1001, {}}, 
 	"Label" 									-> {1002, {}}, 
 	"Neuroname" 								-> {1003, {}}, 
 	{"Matrix", {p1_, p2_}}						:> {1004, {p1, p2}},
 	{"SymmetricMatrix", {p1_, p1_}} 			:> {1005, {p1}},
 	"DisplacementVector" 						-> {1006, {}}, 
 	"Vector" 									-> {1007, {}}, 
 	"Pointset" 									-> {1008, {}}, 
 	"Triangle" 									-> {1009, {}}, 
 	"Quaternion" 								-> {1010, {}}, 
 	"Dimensionless" 							-> {1011, {}}
}];

$ReverseTimingOrder = Dispatch[{
	Undefined 					-> 0, 
	"SequentialIncreasing" 		-> 1, 
 	"SequentialDecreasing" 		-> 2, 
 	"AlternatingIncreasing1" 	-> 3, 
 	"AlternatingDecreasing1" 	-> 4, 
 	"AlternatingIncreasing2" 	-> 5,
  	"AlternatingDecreasing2"	-> 6
}];

$ReverseUnits = Dispatch[{
	"DimensionlessUnit" 	-> 0, 
	"Meters" 				-> 1, 
	"Millimeters" 			-> 2,
	"Micrometers" 			-> 3, 
	"Seconds" 				-> 8, 
	"Milliseconds" 			-> 16, 
 	"Microseconds" 			-> 24, 
 	"Hertz" 				-> 32, 
 	"PartsPerMillion" 		-> 40, 
 	("Radians")/("Seconds")	-> 48
}];

$TimingOrder = Dispatch[{
	0 	-> Undefined, 
	1 	-> "SequentialIncreasing",
	2	-> "SequentialDecreasing",
	3	-> "AlternatingIncreasing1",
	4	-> "AlternatingDecreasing1",
	5	-> "AlternatingIncreasing2",
	6	-> "AlternatingDecreasing2"
}];

$TypeChecks = Dispatch[{
	Undefined 					-> NumberQ,
	"Byte" 						-> IntegerQ,
	"Integer16" 				-> IntegerQ,
	"Integer32" 				-> IntegerQ,
	"Real32" 					-> Internal`RealValuedNumberQ,
	"Complex64" 				-> NumberQ,
	"Real64" 					-> Internal`RealValuedNumberQ, 
	{"Byte", "Byte", "Byte"} 	-> IntegerQ,
	"Integer8" 					-> IntegerQ,
	"UnsignedInteger16" 		-> IntegerQ,
	"UnsignedInteger32" 		-> IntegerQ,
	"Integer64" 				-> IntegerQ,
	"UnsignedInteger64" 		-> IntegerQ,
	"Real128" 					-> Internal`RealValuedNumberQ,
	"Complex128" 				-> NumberQ,
	"Complex256" 				-> NumberQ
}];

$TypeSizes = Dispatch[{
	0 		-> Undefined,
	2 		-> 8, 
	4 		-> 16, 
	8 		-> 32, 
	16 		-> 32, 
	32 		-> 64, 
	64 		-> 64,
	128 	-> 24, 
	256 	-> 8, 
	512 	-> 16, 
	768 	-> 32, 
	1024 	-> 32, 
	1280 	-> 32, 
	1536 	-> 128, 
	1792 	-> 128, 
	2048 	-> 256
}]; 

$Units = Dispatch[{
	0 	-> "DimensionlessUnit", 
	1 	-> "Meters",
	2	-> "Millimeters",
	3	-> "Micrometers",
	8	-> "Seconds",
	16	-> "Milliseconds",
	24	-> "Microseconds",
	32	-> "Hertz",
	40	-> "PartsPerMillion",
	48	-> "Radians"/"Seconds"
}];

(*Extended headers*)
$Templates[1] = {
	{_, _,  _, _, _, _, {_, _, _},{5,_, _, _, _, _, _, _},_, _, _, 1007, _, _, _, {_, _, _, _, _, _, _, _}, _, _, _, _, _, {_, _},_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, {_, _, _, _},{_, _, _, _},{_, _, _, _}, "MiND", _, {val_ /; Not[PossibleZeroQ[val]], _,  _, _}} :> "MiND",
	_ -> Undefined
};
$Templates[2] = {
	{_, _, _, _, {5, _, _, _, _, _, _, _}, _, _, _, {_, _, _, _, _, _, _, _}, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, {_, _, _, _}, {_, _, _, _}, {_, _, _, _}, _, {_, _}, 1007, "MiND", {_, _, _}, _, {1, 0, 0, 0}} :> "MiND",
	_ -> Undefined
};

$ReverseTemplates = {
	{___, "Extensions" -> {"MiND" -> _}, ___} -> "MiND",
	{___} -> Undefined
};

(*Supporting functions*)
JoinPrintableCharacters[list_List] := StringReplace[StringJoin[StringCases[Cases[list, _?StringQ], character_/;MemberQ[CharacterRange[" ", "~"], character]]], (StartOfString ~~ Whitespace) | (Whitespace ~~ EndOfString) :> ""];

PrimaryElement[expr_?AtomQ] := expr;
PrimaryElement[expr_] := First[expr];

QuaternionToRotationMatrix[{a_?NumericQ, b_?NumericQ, c_?NumericQ, d_?NumericQ}] := {
	{a^2 + b^2 - c^2 - d^2, 2 b c - 2 a d, 2 b d + 2 a c},
	{2 b c + 2 a d, a^2 + c^2 - b^2 - d^2, 2 c d - 2 a b},
	{2 b d - 2 a c, 2 c d + 2 a b, a^2 + d^2 - c^2 - b^2}
};
QuaternionVectorToRotationMatrix[{b_?NumericQ, c_?NumericQ, d_?NumericQ}] := QuaternionToRotationMatrix[{Sqrt[1 - b^2 - c^2 - d^2], b, c, d}];

RotationMatrixToQuaternion[{{r11_?NumericQ, r12_?NumericQ, r13_?NumericQ}, {r21_?NumericQ, r22_?NumericQ, r23_?NumericQ}, {r31_?NumericQ, r32_?NumericQ, r33_?NumericQ}}] := Module[{trace = 1. + r11 + r22 + r33, a, b, c, d, x, y, z},
  
  	(*Based on nifti1_io.c*)
  	If[trace > .5,
   
	   a = .5 Sqrt[trace];
	   b = .25 (r32 - r23)/a;
	   c = .25 (r13 - r31)/a;
	   d = .25 (r21 - r12)/a,
		   
	   x = 1. + r11 - (r22 + r33); 
	   y = 1. + r22 - (r11 + r33); 
	   z = 1. + r33 - (r11 + r22);
		   
	   Which[
		    
		    x > 1.,
		    b = .5 Sqrt[x];
		    a = .25 (r32 - r23)/b;
		    c = .25 (r12 + r21)/b;
		    d = .25 (r13 + r31)/b,
			    
		    y > 1.,
		    c = .5 Sqrt[y];
		    a = .25 (r13 - r31)/c;
		    b = .25 (r12 + r21)/c;
		    d = .25 (r23 + r32)/c,
			    
		    True,
		    d = .5 Sqrt[z];
		    a = .25 (r21 - r12)/d;
		    b = .25 (r13 + r31)/d;
		    c = .25 (r23 + r32)/d;
			    
	   ];
		   
	   If[a < 0., {a, b, c, d} *= -1.]
		   
	];
		  
	{a, b, c, d}
		  
];
RotationMatrixToQuaternionVector[r : {{_?NumericQ, _?NumericQ, _?NumericQ}, {_?NumericQ, _?NumericQ, _?NumericQ}, {_?NumericQ, _?NumericQ, _?NumericQ}}] := Rest[RotationMatrixToQuaternion[r]];

SphericalToCartesianCoordinates[phi_?NumericQ,theta_?NumericQ] := -{Sin[phi] Cos[theta], Sin[phi] Sin[theta], Cos[phi]};
SphericalToCartesianCoordinates[_, _] := {0., 0., 0.};

CartesianToSphericalCoordinates[{0 | 0., 0 | 0., 0 | 0.}] = {Indeterminate, Indeterminate};
CartesianToSphericalCoordinates[{0 | 0., 0 | 0., z : 1 | 1. | -1 | -1.}] = {Pi (1 - Sign[z]) / 2, 0};
CartesianToSphericalCoordinates[{x_?NumberQ, y_?NumberQ, z_?NumberQ}] := {ArcCos[-z], ArcTan[-x, -y]};

ScalingTransformationQ[expr_] := MatchQ[expr, TransformationFunction[{{_?Positive, 0 | 0., 0 | 0., 0 | 0.}, {0 | 0., _?Positive, 0 | 0., 0 | 0.}, {0 | 0., 0 | 0., _?Positive, 0 | 0.}, {0 | 0., 0 | 0., 0 | 0., 1 | 1.}}]]

AffineTransformationQ[expr_] := MatchQ[expr, TransformationFunction[mat : {{_?NumberQ, _?NumberQ, _?NumberQ, _?NumberQ}, {_?NumberQ, _?NumberQ, _?NumberQ, _?NumberQ}, {_?NumberQ, _? NumberQ, _?NumberQ, _?NumberQ}, {0 | 0., 0 | 0., 0 | 0., 1 | 1.}}] /; MatrixRank[mat] == 4]

DetectDataType[list : {_Integer?NonNegative ..}] := Module[{max = Max[list]},

  	First@Extract[{"Byte", "UnsignedInteger16", "UnsignedInteger32"}, Position[{255, 65535, 4294967295}, size_ /; size > max, 1, 1]]

];
DetectDataType[list : {_Integer ..}] := Module[{min = Min[list], max = Max[list]}, 
	
	First@Extract[{"Integer8", "Integer16", "Integer32"},
		
		Max[Position[{127, 32767, 2147483647}, size_ /; size > max, 1, 1],Position[{-128, -32768, -2147483648}, size_ /; size < min, 1, 1]]
		
    ]
  
];
DetectDataType[list : {(_Real | _Rational) ..}] := "Real32";
DetectDataType[list : {_Complex ..}] := "Complex64";
DetectDataType[___] := "Real32";

IntegerChop = # + Chop[#2 - #]&[Round@#, #] &

(*Default import*)
Options[NIfTIDefaultImport] = {"Channel" -> Null, "Compact" -> True, "ExtensionParsing" -> False};

NIfTIDefaultImport[filename_String, opts : OptionsPattern[]] := Module[
	{
		compact = OptionValue["Compact"],
		parse	= OptionValue["ExtensionParsing"]
	},
	
	(*Check options*)
	If[!Element[compact, Booleans], Message[Import::opttf, "\"Compact\"", compact]; Return[$Failed, Module]];
	If[!Element[parse, Booleans], Message[Import::opttf, "\"ExtensionParsing\"", parse]; Return[$Failed, Module]];

	(*Return import*)
	With[{metainformation = "MetaInformation" /. NIfTIMetaInformationImport[filename, opts]}, {"MetaInformation" -> metainformation, "Data" -> NIfTIDataImport[filename, metainformation, opts]}]

];

(*MetaInformation import*)
NIfTIMetaInformationImport[filename_String, opts : OptionsPattern[NIfTIDefaultImport]] := Module[
	{
		channel	= OptionValue["Channel"],  
		
		hdrfile
	},
	
	(*Check extension header file*)
	hdrfile = If[StringMatchQ[FileExtension[filename], "img", IgnoreCase -> True],
		First[System`ConvertersDump`Decode[StringReplace[channel, RegularExpression["img(?!.*img)"] -> "hdr", IgnoreCase->True], {Automatic}]],
		filename
	];
	
	{"MetaInformation" -> NIfTIMetaInformationImport[hdrfile,
	
		(**)
		Switch[
			
			Block[{stream},
		
				Internal`WithLocalSettings[
					
					stream = OpenRead[hdrfile, BinaryFormat -> True],
					
					Quiet[BinaryRead[stream, "Integer32"]],
					
					Close[stream]
					
				]
				
			],
			
			348 | 1543569408,
			1,
			
			540 | 469893120,
			2,
			
			_,
			Null
					
		
		],
		
		opts
		
	]}
	
];

NIfTIMetaInformationImport[hdrfile_String, 1, OptionsPattern[NIfTIDefaultImport]] := Module[
	{
		headertypes 	= {{"Integer32", 1}, {"Character8", 10}, {"Character8", 18}, {"Integer32", 1}, {"Integer16", 1}, {"Character8", 1}, {"Byte", 1}, {"Integer16", 8}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Integer16", 1}, {"Integer16", 1}, {"Integer16", 1}, {"Integer16", 1}, {"Real32", 8}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Integer16", 1}, {"Byte", 1}, {"Byte", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Integer32", 1}, {"Integer32", 1}, {"Character8", 80}, {"Character8", 24}, {"Integer16", 1}, {"Integer16", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 4},{"Real32", 4}, {"Real32", 4}, {"Character8", 16}, {"Character8", 4}, {"Byte", 4}}, 
		byteordering	= $ByteOrdering,
		parse 			= OptionValue["ExtensionParsing"], 
		
		stream, header, extheaderid
	},
	
	Internal`WithLocalSettings[
		
		stream = OpenRead[hdrfile, BinaryFormat -> True],
	
		(*Correct for endianness*)
		If[BinaryRead[stream, "Integer32", ByteOrdering -> byteordering] != 348, byteordering *= -1];
	
		(*Reset stream position*)
		SetStreamPosition[stream, 0];
	
		(*Read NIfTI header*)
		header 	= Check[BinaryRead[stream, ConstantArray[#[[1]], {#[[2]]}] & /@ headertypes, ByteOrdering -> byteordering], Message[Import::errstruct, "\"MetaInformation\". Errors generated while reading header."]; Return[$Failed, Module]];
		
		(*Check header length*)
		If[header[[43, -1]] === EndOfFile, Message[Import::errstruct, "\"MetaInformation\". The header is incomplete."]; Return[$Failed, Module]];
		
		(*Subparsing*)
		header[[7]] 	= Most[FromDigits[Reverse[#], 2]& /@ Partition[Reverse[IntegerDigits[header[[7, 1]], 2, 8]], 2]];
		header[[22]]	= {FromDigits[Reverse[#], 2]& /@ Partition[Reverse[IntegerDigits[header[[22, 1]], 2, 8]][[;; 6]], 3]{1, 8}};
		header 			= Thread[{header, headertypes}] /. {{val_, {"Character8", len_}} :> JoinPrintableCharacters[val], {{val_}, {_, 1}} :> val, {val_, {_, _}} :> val};
		
		(*Check header*)
		If[
			Not[VectorQ[{
				Equivalent[header[[13]] != 0, (header[[13]] /. $TypeSizes) == header[[14]]] /. False :> (Message[Import::errstruct, "\"MetaInformation\". \"BitDepth\" is incompatible with \"DataType\"."]; False),
				Equivalent[NonNegative[header[[15]]], header[[20]] >= header[[15]]] /. False :> (Message[Import::errstruct, "\"MetaInformation\". \"SliceRange\" is not specified correctly."]; False),
				StringMatchQ[header[[43]], "n" ~~ {"+", "i"} ~~ CharacterRange["1", "9"]] /. False :> (Message[Import::errstruct, "\"MetaInformation\". The magic string that defines \"SeparateHeader\" and \"NIfTIVersion\" is not specified correctly."]; False),
				If[header[[8, 1]] == 5 && 2 <= header[[12]] <= 24, ((header[[8, 6]] == 1) || Block[{intent = header[[12]] /. $IntentCodes}, header[[8, 6]] == 1 + intent[[2]]]), True] /. False :> (Message[Import::errstruct, "\"MetaInformation\". \"Dimensions\" is incompatible with \"Intent\"."]; False)
			}, TrueQ]], 
			Return[$Failed, Module]
		];
	
		(*Label header*)
		header = {
			"StatisticalParametricDataSet"	-> 2 <= header[[12]] <= 24,
			"Version" 						-> 1,
			"ExtensionOffset" 				-> 352,
			"ByteOrdering" 					-> byteordering, 
			"HeaderLength" 					-> 348,
			"NIfTIUnused" 					-> header[[{2,3,4,5,6,27,28}]],
			"SliceOrdering" 				-> If[header[[7]] == {0, 0, 0}, Undefined, MapThread[Rule,{{"FrequencyEncoding", "PhaseEncoding", "SliceEncoding"}, header[[7]] /. $Directions}]],
			"NumberOfDimensions" 			-> header[[8, 1]],
			"Dimensions" 					-> header[[8, 2 ;; header[[8, 1]] + 1]],
			"Intent" 						-> Block[{intent = header[[12]] /. $IntentCodes}, If[(2 <= header[[12]] <= 24) && (header[[8, 6]] > 1), intent[[1]], intent[[1]] @@ header[[9 ;; 8 + intent[[2]]]]]],
			"DataType" 						-> (header[[13]] /.  $DataTypes),
			"BitDepth" 						-> header[[14]],
			"SliceRange" 					-> If[Negative[header[[15]]] || header[[15]] >= header[[20]] || header[[20]] == 0, Undefined, {header[[15]], header[[20]]}],
			"SpatialUnit" 					-> header[[22, 1]] /. $Units,
			"TemporalUnit" 					-> header[[22, 2]] /. $Units,
			"DataOffset" 					-> Round[header[[17]]],
			"DataScaling" 					-> If[(header[[18]] == 1. && header[[19]] == 0.) || header[[18]] == 0., Identity, Evaluate[header[[18]] # + header[[19]]]&],
			"TimingOrder" 					-> header[[21]] /. $TimingOrder,
			"IntensityRange" 				-> If[header[[23]] == header[[24]], Undefined, {header[[24]], header[[23]]}],
			"SliceAcquisitionDuration" 		-> If[header[[25]] == 0., Undefined, header[[25]]],
			"TemporalTransformation" 		-> AffineTransform[{{{header[[16, 5]]}}, {header[[26]]}}],
			"Description" 					-> If[header[[29]] == "", Undefined, header[[29]]],
			"AuxiliaryFileName" 			-> If[header[[30]] == "", Undefined, header[[30]]],
			"SpatialTransformation" 		-> Block[{transforms = Join[
					Switch[
						header[[31]],
						0,
						{"ArbitraryCoordinates" -> ScalingTransform[header[[16, 2 ;; 4]]]},
						1,
						{"ScannerBasedAnatomicalCoordinates" -> AffineTransform[{QuaternionVectorToRotationMatrix[header[[33;;35]]].DiagonalMatrix[{1,1,If[header[[16,1]] == -1, -1, 1]}header[[16, 2 ;; 4]]], header[[36;;38]]}]},
						2 | 3 | 4,
						Message[Import::general, "Warning: primary coordinate system not properly specified."];
						{(header[[32]] /. $Forms) -> AffineTransform[{QuaternionVectorToRotationMatrix[header[[33;;35]]].DiagonalMatrix[{1,1,If[header[[16,1]] == -1, -1, 1]}header[[16, 2 ;; 4]]], header[[36;;38]]}]},
						_,
						Message[Import::errstruct, "\"MetaInformation\". Unknown atlas code in the coordinate system specification."]; Return[$Failed, Module]
					],
					Switch[
						header[[32]],
						0,
						{},
						1,
						Message[Import::general, "Warning: secondary coordinate system not properly specified."];
						{"ScannerBasedAnatomicalCoordinates" -> AffineTransform[{header[[39 ;; 41, ;; 3]], header[[39 ;; 41, 4]]}]},
						2 | 3 | 4,
						{(header[[32]] /. $Forms) -> AffineTransform[{header[[39 ;; 41, ;; 3]], header[[39 ;; 41, 4]]}]},
						_,
						Message[Import::errstruct, "\"MetaInformation\". Unknown atlas code in the coordinate system specification."]; Return[$Failed, Module]
					]					
				]},
				
				If[(header[[31]] == header[[32]] != 0) && DeleteDuplicates[transforms] === transforms, 
					
					If[header[[31]] == 1,	
						
						Message[Import::general, Row[{"Warning: Inconsistent coordinate systems specified. Disregarding the secondary coordinate system ", transforms[[2, 2]], "."}]];
						
						transforms[[{1}]],
						
						Message[Import::general, Row[{"Warning: Inconsistent coordinate systems specified. Disregarding the primary coordinate system ", transforms[[1, 2]], "."}]];
						
						transforms[[{2}]]
						
					],
					
					transforms
					
				]
				
			],
			"DefaultCoordinates" 			-> If[
				header[[32]] > 1,
				header[[32]] /. $Forms,
				header[[31]] /. $Forms
			],			
			"IntentName" 					-> If[header[[42]] == "", Undefined, header[[42]]],
			"SeparateHeader" 				-> StringMatchQ[StringTake[header[[43]], {2}], "i"],
			"NIfTIVersion" 					-> ToExpression[StringTake[header[[43]], {3}]],
			"ExtendedHeader" 				-> If[Not[Or[PossibleZeroQ[header[[44, 1]]], header[[44, 1]] === EndOfFile]], extheaderid = header /. $Templates[1], extheaderid = None],
			"ExtensionParsing"				-> parse
		};
	
		(*Check NIfTI version*)
		With[{version = "NIfTIVersion" /. header}, If[version != 1, Message[Import::formvers, ToString[version]]]];
	
		(*Return metainformation*)
		Sort[Join[header, NIfTIExtendedMetaInformationImport[stream, header, extheaderid]]],
		
		Close[stream]
		
	]

];

NIfTIMetaInformationImport[hdrfile_String, 2, OptionsPattern[NIfTIDefaultImport]] := Module[
	{
		headertypes 	= {{"Integer32", 1}, {"Character8", 8}, {"Integer16", 1}, {"Integer16", 1}, {"Integer64", 8}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 8}, {"Integer64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Integer64", 1}, {"Integer64", 1}, {"Character8", 80}, {"Character8", 24}, {"Integer32", 1}, {"Integer32", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 4}, {"Real64", 4}, {"Real64", 4}, {"Integer32", 1}, {"Integer32", 1}, {"Integer32", 1}, {"Character8", 16}, {"Byte", 1}, {"Character8", 15}, {"Byte", 4}}, 
		byteordering	= $ByteOrdering, 
		parse 			= OptionValue["ExtensionParsing"],
		
		stream, header, extheaderid
	},
	
	Internal`WithLocalSettings[
		
		stream = OpenRead[hdrfile, BinaryFormat -> True],
	
		(*Correct for endianness*)(**)
		If[BinaryRead[stream, "Integer32", ByteOrdering -> byteordering] != 540, byteordering *= -1];
	
		(*Reset stream position*)(**)
		SetStreamPosition[stream, 0];
	
		(*Read NIfTI header*)
		header 	= Check[BinaryRead[stream, ConstantArray[#[[1]], {#[[2]]}] & /@ headertypes, ByteOrdering -> byteordering], Message[Import::errstruct, "\"MetaInformation\". Errors generated while reading header."]; Return[$Failed, Module]];
		
		(*Check header length*)
		If[header[[37, -1]] === EndOfFile, Message[Import::errstruct, "\"MetaInformation\". The header is incomplete."]; Return[$Failed, Module]];
		
		(*Subparsing*)
		header[[36]] 	= Most[FromDigits[Reverse[#], 2]& /@ Partition[Reverse[IntegerDigits[header[[36, 1]], 2, 8]], 2]];
		header[[33]]	= {FromDigits[Reverse[#], 2]& /@ Partition[Reverse[IntegerDigits[header[[33, 1]], 2, 32]][[;; 6]], 3]{1, 8}};
		header 			= Thread[{header, headertypes}] /. {{val_, {"Character8", len_}} :> JoinPrintableCharacters[val], {{val_}, {_, 1}} :> val, {val_, {_, _}} :> val};
				
		(*Check header*)
		If[
			Not[VectorQ[{
				Equivalent[header[[3]] != 0, (header[[3]] /. $TypeSizes) == header[[4]]] /. False :> (Message[Import::errstruct, "\"MetaInformation\". \"BitDepth\" is incompatible with \"DataType\"."]; False),
				Equivalent[NonNegative[header[[17]]], header[[18]] >= header[[17]]] /. False :> (Message[Import::errstruct, "\"MetaInformation\". \"SliceRange\" is not specified correctly."]; False),
				StringMatchQ[header[[2]], "n" ~~ {"+", "i"} ~~ CharacterRange["1", "9"]] /. False :> (Message[Import::errstruct, "\"MetaInformation\". The magic string that defines \"SeparateHeader\" and \"NIfTIVersion\" is not specified correctly."]; False),
				If[header[[5, 1]] == 5 && 2 <= header[[34]] <= 24, ((header[[5, 6]] == 1) || Block[{intent = header[[34]] /. $IntentCodes}, header[[5, 6]] == 1 + intent[[2]]]), True] /. False :> (Message[Import::errstruct, "\"MetaInformation\". \"Dimensions\" is incompatible with \"Intent\"."]; False)
			}, TrueQ]], 
			Return[$Failed, Module]
		];
		
		(*Label header*)
		header = {
			"StatisticalParametricDataSet"	-> 2 <= header[[34]] <= 24,
			"Version" 						-> 2,
			"ExtensionOffset" 				-> 544,
			"ByteOrdering" 					-> byteordering, 
			"HeaderLength" 					-> 540,
			"NIfTIUnused" 					-> header[[37]],
			"SliceOrdering" 				-> If[header[[36]] == {0, 0, 0}, Undefined, MapThread[Rule,{{"FrequencyEncoding", "PhaseEncoding", "SliceEncoding"}, header[[36]] /. $Directions}]],
			"NumberOfDimensions" 			-> header[[5, 1]],
			"Dimensions" 					-> header[[5, 2 ;; header[[5, 1]] + 1]],
			"Intent" 						-> Block[{intent = header[[34]] /. $IntentCodes}, If[(2 <= header[[34]] <= 24) && (header[[5, 6]] > 1), intent[[1]], intent[[1]] @@ header[[6 ;; 5 + intent[[2]]]]]],
			"DataType" 						-> (header[[3]] /. $DataTypes),
			"BitDepth" 						-> header[[4]],
			"SliceRange" 					-> If[Negative[header[[17]]] || header[[17]] >= header[[18]] || header[[18]] == 0, Undefined, {header[[17]], header[[18]]}],
			"SpatialUnit" 					-> header[[33, 1]] /. $Units,
			"TemporalUnit" 					-> header[[33, 2]] /. $Units,
			"DataOffset" 					-> Round[header[[10]]],
			"DataScaling" 					-> If[(header[[11]] == 1. && header[[12]] == 0.) || header[[11]] == 0., Identity, Evaluate[header[[11]] # + header[[12]]]&],
			"TimingOrder" 					-> header[[32]] /. $TimingOrder,
			"IntensityRange" 				-> If[header[[13]] == header[[14]], Undefined, {header[[14]], header[[13]]}],
			"SliceAcquisitionDuration" 		-> If[header[[15]] == 0., Undefined, header[[25]]],
			"TemporalTransformation" 		-> AffineTransform[{{{header[[9, 5]]}}, {header[[16]]}}],
			"Description" 					-> If[header[[19]] == "", Undefined, header[[19]]],
			"AuxiliaryFileName" 			-> If[header[[20]] == "", Undefined, header[[20]]],
			"SpatialTransformation" 		-> Block[{transforms = Join[
					Switch[
						header[[21]],
						0, 
						{"ArbitraryCoordinates" -> ScalingTransform[header[[9, 2 ;; 4]]]},
						1, 
						{"ScannerBasedAnatomicalCoordinates" -> AffineTransform[{QuaternionVectorToRotationMatrix[header[[23;;25]]].DiagonalMatrix[{1,1,If[header[[9,1]] == -1, -1, 1]}header[[9, 2 ;; 4]]], header[[26;;28]]}]},
						2 | 3 | 4,
						Message[Import::general, "Warning: primary coordinate system not properly specified."];
						{(header[[22]] /. $Forms) -> AffineTransform[{QuaternionVectorToRotationMatrix[header[[23;;25]]].DiagonalMatrix[{1,1,If[header[[9,1]] == -1, -1, 1]}header[[9, 2 ;; 4]]], header[[26;;28]]}]}, 
						_,
						Message[Import::errstruct, "\"MetaInformation\". Unknown atlas code in the coordinate system specification."]; Return[$Failed, Module]
					],
					Switch[
						header[[22]],
						0,
						{},
						1,
						Message[Import::general, "Warning: secondary coordinate system not properly specified."];
						{"ScannerBasedAnatomicalCoordinates" ->  AffineTransform[{header[[29 ;; 31, ;; 3]], header[[29 ;; 31, 4]]}]}, 
						2 | 3 | 4,
						{(header[[22]] /. $Forms) -> AffineTransform[{header[[29 ;; 31, ;; 3]], header[[29 ;; 31, 4]]}]}, 
						_,
						Message[Import::errstruct, "\"MetaInformation\". Unknown atlas code in the coordinate system specification."]; Return[$Failed, Module]
					]
				]},
				
				If[(header[[21]] == header[[22]] != 0) && DeleteDuplicates[transforms] === transforms, 
						
					If[header[[21]] == 1,	
						
						Message[Import::general, Row[{"Warning: Inconsistent coordinate systems specified. Disregarding the secondary coordinate system ", transforms[[2, 2]], "."}]];
						
						transforms[[{1}]],
						
						Message[Import::general, Row[{"Warning: Inconsistent coordinate systems specified. Disregarding the primary coordinate system ", transforms[[1, 2]], "."}]];
						
						transforms[[{2}]]
						
					],
					
					transforms
					
				]
							
			],
			"DefaultCoordinates" 			-> If[
				header[[22]] > 1,
				header[[22]] /. $Forms,
				header[[21]] /. $Forms
			],			
			"IntentName" 					-> If[header[[35]] == "", Undefined, header[[35]]],
			"SeparateHeader" 				-> StringMatchQ[StringTake[header[[2]], {2}], "i"],
			"NIfTIVersion" 					-> ToExpression[StringTake[header[[2]], {3}]],
			"ExtendedHeader" 				-> (If[Not[Or[PossibleZeroQ[header[[38, 1]]], header[[38, 1]] === EndOfFile]], extheaderid = header /. $Templates[2], extheaderid = None]),
			"ExtensionParsing"				-> parse
		};
	
		(*Check NIfTI version*)
		With[{version = "NIfTIVersion" /. header}, If[version != 2, Message[Import::formvers, ToString[version]]]];
	
		(*Return metainformation*)
		Sort[Join[header, NIfTIExtendedMetaInformationImport[stream, header, extheaderid]]],
		
		Close[stream]
		
	]

];

NIfTIMetaInformationImport[___] := (Message[Import::errstruct, "\"MetaInformation\". Unrecognized format."]; Return[$Failed])

NIfTIExtendedMetaInformationImport[_InputStream, _List, None] := {"Extensions" -> {}};

NIfTIExtendedMetaInformationImport[stream_InputStream, header_List, Undefined] := Module[
	{
		byteordering	= "ByteOrdering" /. header,
		dataoffset 		= "DataOffset" /. header,
		sepheaderbool	= "SeparateHeader" /. header,
		parse			= "ExtensionParsing" /. header,
		
		ext
	},
	
	(*Read official and unknown header extensions*)
	Block[{code = {}},
			
		ext = If[sepheaderbool,
			
			Flatten[
				Reap[
					While[FreeQ[code = BinaryRead[stream, {"Integer32", "Integer32"}, ByteOrdering -> byteordering], EndOfFile, 1],
						Sow[{code[[2]] /. $ECodes, NIfTIExtendedMetaInformationImport[stream, header, If[parse, code, {code[[1]], 0}]]}]
					]
				][[2]],
				1
			],
			
			Flatten[
				Reap[
					While[StreamPosition[stream] < dataoffset,
						If[StreamPosition[stream] < dataoffset - 8, 
							
							code = BinaryRead[stream, {"Integer32", "Integer32"}, ByteOrdering -> byteordering];
							If[StreamPosition[stream] + code[[1]] - 8 <= dataoffset,
								Sow[{code[[2]] /. $ECodes, NIfTIExtendedMetaInformationImport[stream, header, If[parse, code, {code[[1]], 0}]]}],
								Message[Import::errstruct, "\"MetaInformation\". The extended header is incomplete."]
							],
							
							Message[Import::errstruct, "\"MetaInformation\". Parts at the end of the extended header are not specified correctly."];
							Skip[stream, dataoffset - StreamPosition[stream]]
							
						]
					]
				][[2]],
				1
			]
				
		]			

	];
	
	(*Return header extensions*)
	{"Extensions" -> ext}
	
];

NIfTIExtendedMetaInformationImport[stream_InputStream, header_List, {size_Integer, 2_Integer}] := Module[
	{
		pos 			= StreamPosition[stream],
		byteordering	= "ByteOrdering" /. header,
		dataoffset 		= "DataOffset" /. header,
		sepheaderbool	= "SeparateHeader" /. header,
		skiplength 		= size - 8,
		
		extheader
	},
	
	If[Not[sepheaderbool] && pos + skiplength > dataoffset, 
		
		Skip[stream, "Byte", dataoffset - pos];
		Message[Import::errstruct, "\"MetaInformation\". Extended header apparently specified beyond \"DataOffset\"."];
		Return[Undefined, Module], 

		extheader = StringJoin[BinaryReadList[stream, "Character8", skiplength, ByteOrdering -> byteordering]];
		Return[ImportString[extheader, "DICOM"] /. $Failed -> extheader, Module]
		
	];
	
];

(*Import as XML (AFNI / XCEDE / JIM / FWDS)*)
NIfTIExtendedMetaInformationImport[stream_InputStream, header_List, {size_Integer, 4 | 8 | 10 | 12}] := Module[
	{
		pos 			= StreamPosition[stream],
		byteordering	= "ByteOrdering" /. header,
		dataoffset 		= "DataOffset" /. header,
		sepheaderbool	= "SeparateHeader" /. header,
		skiplength 		= size - 8,
		
		extheader
	},
	
	If[Not[sepheaderbool] && pos + skiplength > dataoffset, 
		
		Skip[stream, "Byte", dataoffset - pos];
		Message[Import::errstruct, "\"MetaInformation\". Extended header apparently specified beyond \"DataOffset\"."];
		Return[Undefined, Module], 

		extheader = StringJoin[BinaryReadList[stream, "Character8", skiplength, ByteOrdering -> byteordering]];
		Return[ImportString[extheader, "XML"] /. $Failed -> extheader, Module]
		
	];
	
];

(*Import extended header as string (default / Comment)*)
NIfTIExtendedMetaInformationImport[stream_InputStream, header_List, {size_Integer, code_Integer}] := Module[
	{
		pos 			= StreamPosition[stream],
		byteordering	= "ByteOrdering" /. header,
		dataoffset 		= "DataOffset" /. header,
		sepheaderbool	= "SeparateHeader" /. header,
		skiplength 		= size - 8,
		
		extheader
	},
	
	If[Not[sepheaderbool] && pos + skiplength > dataoffset, 
		
		Skip[stream, "Byte", dataoffset - pos];
		Message[Import::errstruct, "\"MetaInformation\". Extended header apparently specified beyond \"DataOffset\"."];
		Return[Undefined, Module], 

		extheader = StringJoin[BinaryReadList[stream, "Character8", skiplength, ByteOrdering -> byteordering]];
		Return[extheader, Module]
		
	];
	
];

NIfTIExtendedMetaInformationImport[stream_InputStream, header_List, "MiND"] := Module[
	{
		ecodestructure 	= {{esize_, 18} :> ConstantArray["Character8", {esize - 8}],{esize_, 20} -> "Real64", {esize_, 22} -> {"Real32", "Real32"}, {esize_, 24} -> {"Integer32", "Integer32"}, {esize_, 26} -> {"Integer32", "Integer32"}},
		dataoffset		= "DataOffset" /. header,
		byteordering 	= "ByteOrdering" /. header,
		sepheaderbool   = "SeparateHeader" /. header,
		
		vals, extheader
	},

	(*Read MiND header*)
	Check[
		
		Block[{code = {}},
				
			vals = If[sepheaderbool,
				
				Reap[
					While[FreeQ[code, EndOfFile, 1],
						code = BinaryRead[stream, {"Integer32", "Integer32"}, ByteOrdering -> byteordering];
						Sow[BinaryRead[stream, code /. ecodestructure, ByteOrdering -> byteordering]]
					]
				][[2, 1]],
				
				Reap[
					While[StreamPosition[stream] < dataoffset,
						code = BinaryRead[stream, {"Integer32", "Integer32"}, ByteOrdering -> byteordering];
						Sow[BinaryRead[stream, code /. ecodestructure, ByteOrdering -> byteordering]]
					]
				][[2, 1]]
					
			]			

		], 
	
		Message[Import::errstruct, "MiND header. Errors generated while reading header."]; 
		Return[$Failed, Module]
	
	];
	
	(*Subparsing*)
	extheader 			= Partition[SplitBy[vals, Count[#, _?StringQ] != 0&], 2];(*Split DWIs*)
	extheader[[All, 1]] = JoinPrintableCharacters /@ Flatten[extheader[[All, 1]], 1];(*Convert IDs to string*)
	extheader 			= extheader /. {"RAWDWI", x__} :> {"RAWDWI", Partition[x, 2]};(*Partition Raw DWI headers*)
	extheader 			= extheader /. {{"RAWDWI", m_} :> {"RAWDWI", Transpose[{Quantity[m[[All, 1]], "Seconds" / ("Millimeters"^2)], SphericalToCartesianCoordinates@@@m[[All, 2]]}]}, {"DISCSPHFUNC", x_List} :> {"DISCSPHFUNC", SphericalToCartesianCoordinates@@@x}};(*Convert the zenith-azimuth angles to unit vectors*)

	(*Check header*)
	If[
		Not[Total[Length /@ extheader[[All, 2]]] == Last["Dimensions" /. header]], 
		Message[Import::errstruct, "MiND header. The information specified in the MiND extended header is incompatible with \"Dimensions\"."]; Return[$Failed]
	];

	(*Label header*)
	{"Extensions" -> {"MiND" -> extheader}}
	
];

(*Data import*)
NIfTIDataImport[filename_String, metainformation_List, opts : OptionsPattern[NIfTIDefaultImport]] := Module[
	{
		channel = OptionValue["Channel"],
		compact = OptionValue["Compact"],
		
		imgfile, byteordering, dim, offset, datatype, stream, data, datascaling
	},
	
	(*Check metainformation*)
	If[metainformation === $Failed, Return[$Failed]];

	(*Check extension image file*)
	imgfile = If[StringMatchQ[FileExtension[filename], "hdr", IgnoreCase -> True],
		First[System`ConvertersDump`Decode[StringReplace[channel, RegularExpression["hdr(?!.*hdr)"] -> "img", IgnoreCase->True], {Automatic}]],
		filename
	];

	(*Read metainformation*)
	byteordering 	= "ByteOrdering" /. metainformation;
	dim 			= "Dimensions" /. metainformation;
	offset 			= "DataOffset" /. metainformation;
	datatype		= "DataType" /. metainformation;
	datascaling 	= If[datatype =!= {"Byte","Byte","Byte"}, "DataScaling" /. metainformation, Identity];

	(*Check data type*)
	If[datatype === Undefined, Return[$Failed]];
	
	Internal`WithLocalSettings[
		
		stream = OpenRead[imgfile, "BinaryFormat" -> True],
		
		(*Set stream position*)
		Check[SetStreamPosition[stream, offset], Message[Import::errstruct, "NIfTI data. Failed to set stream position to the offset specified by \"DataOffset\"."]; Return[$Failed]];
	
		(*Read data*)
		If[compact,
			data = Check[BinaryReadList[stream, datatype, Times @@ dim, ByteOrdering-> byteordering], Message[Import::errstruct, "NIfTI data. Errors generated while reading data."]; Return[$Failed]];
			If[Length[data] != Times @@ dim, Message[Import::errstruct, "NIfTI data. The data is incomplete."]; Return[$Failed]];
			data = Internal`Deflatten[datascaling @ data, Reverse[dim]],
			data = Check[Apply[Table[datascaling @ BinaryReadList[stream, datatype, First[dim], ByteOrdering-> byteordering], ##] &, Partition[Reverse@Rest@dim, 1]], Message[Import::errstruct, "NIfTI data. Errors generated while reading data. The data may be incomplete."]; Return[$Failed]]
		],
		
		Close[stream]
		
	]
	
];

(*Unparse extensions*)
Attributes[Unparse] = {Listable};

Unparse[{"DICOM", data_}] /; !StringQ[data] := ExportString[data, "DICOM"];
Unparse[{"AFNI" | "XCEDE" | "JIM" | "FWDS", data_}] /; !StringQ[data] := ExportString[data, "XML"];
Unparse[{_, data_String}] := data;

(*Default export*)
Options[NIfTIDefaultExport] = {
	"AuxiliaryFileName" 				-> Undefined,
	"ByteOrdering" 						:> $ByteOrdering,
	"Channel" 							-> Null,
	"Compatibility"						-> None,
	"DataScaling" 						-> Identity,
	"DataType" 							-> "Real32",
	"Description" 						:> StringTake["Created by Wolfram Mathematica " <> ToString[$VersionNumber], UpTo[80]],
	"Extensions"						-> {},
	"IntensityRange"					-> Undefined,
	"Intent" 							-> Undefined,
	"IntentName" 						-> Undefined,
	"SeparateHeader" 					-> False,
	"SliceAcquisitionDuration" 			-> Undefined,
	"SliceOrdering" 					-> Undefined,
	"SliceRange" 						-> Undefined,
	"SpatialTransformation"			 	-> {"ArbitraryCoordinates" -> ScalingTransform[{1, 1, 1}]},
	"SpatialUnit" 						-> "DimensionlessUnit",
	"TemporalTransformation" 			-> ScalingTransform[{1}],
	"TemporalUnit" 						-> "DimensionlessUnit",
	"TimingOrder" 						-> Undefined,
	"Version" 							-> 1
};

NIfTIDefaultExport[filename_, data_, opts : OptionsPattern[]] := Module[
	{
		version = OptionValue["Version"],
		stream,	header
	},

	If[!StringMatchQ[filename, __ ~~ (".nii" | ".nii.gz"), IgnoreCase -> True], Message[Export::general, "Warning: the specified file extension is not generally supported. The suggested file extension is .nii."]];
	If[version =!= 1, Message[Export::general, "Warning: the standard for the specified version number is not yet finalized and subject to changes."]];
	
	(*Check header*)
	If[(header = NIfTICheckOptions[data, version, opts]) === $Failed, Return[$Failed, Module]];
	
	Internal`WithLocalSettings[
		
		stream = OpenWrite[filename, BinaryFormat -> True],	
			
		(*Write header*)
		If[NIfTIWriteHeader[stream, data, header, version, opts] === $Failed, Return[$Failed, Module]];
		
		(*Write data*)
		NIfTIWriteData[stream, data, header, opts],
		
		Close[stream]
		
	]
		
];

NIfTICheckOptions[data_, version : 1 | 2, opts : OptionsPattern[NIfTIDefaultExport]] := Module[
	{
		aux 					= OptionValue["AuxiliaryFileName"],
		byteordering 			= OptionValue["ByteOrdering"], 
		compatibility  			= OptionValue["Compatibility"],
		datascaling 			= OptionValue["DataScaling"],
		datatype 				= OptionValue["DataType"],
		description 			= OptionValue["Description"],
		intensityrange 			= OptionValue["IntensityRange"], 
		intent 					= OptionValue["Intent"], 
		intentname 				= OptionValue["IntentName"], 
		sepheaderbool 			= OptionValue["SeparateHeader"], 
		sliceduration			= OptionValue["SliceAcquisitionDuration"], 
		sliceordering 			= OptionValue["SliceOrdering"],
		slicerange				= OptionValue["SliceRange"], 
		spatialtransformation	= OptionValue["SpatialTransformation"], 
		spatialunit 			= OptionValue["SpatialUnit"], 
		temporaltransformation 	= OptionValue["TemporalTransformation"], 
		temporalunit 			= OptionValue["TemporalUnit"], 
		$TimingOrder 			= OptionValue["TimingOrder"],
		extheaderid 			= Flatten[{opts}] /. $ReverseTemplates,
		
		dim = Dimensions[data],
		ndim = ArrayDepth[data]
	},
	
	(*Check data*)
	If[!ArrayQ[data, _, NumberQ], Message[Export::errstruct, "\"Data\". The data should be an array of numbers."]; Return[$Failed, Module]];
	
	(*Check options*)(**)
	If[MemberQ[{
		(StringQ[aux] && StringLength[aux] <= 24) || aux === Undefined /. False :> Message[Export::nptg, "\"AuxiliaryFileName\"", aux, "string of 24 or fewer characters or Undefined"],
		byteordering === 1 || byteordering === -1 /. False :> Message[Export::byteord, byteordering, "1 or -1"],
		MemberQ[Normal[$CompatibilityModes][[All, 1]], compatibility] /. False :> Message[Export::nptg, "\"Compatibility\"", compatibility, "member of " <> ToString[Normal[$CompatibilityModes][[All, 1]]]],
		MatchQ[datascaling, Identity | ((Plus | Minus)[_ ? NumberQ #, _ ? NumberQ] &)] /. False :> Message[Export::nptg, "\"DataScaling\"", datascaling, "linear pure function or Identity"],
		(datatype == Automatic || MemberQ[Normal[$ReverseDataTypes][[2 ;;, 1]], datatype]) /. False :> Message[Export::nptg, "\"DataType\"", datatype, "member of " <> ToString[Normal[$ReverseDataTypes][[2 ;;, 1]]] <> " or Automatic"],
		(StringQ[description] && StringLength[description] <= 80) || description === Undefined /. False :> Message[Export::nptg, "\"Description\"", description, "string of 80 or fewer characters or Undefined"],
		intensityrange === Undefined || (MatchQ[intensityrange, {_ ? NumberQ, _ ? NumberQ}]) /. False :> Message[Export::nptg, "\"IntensityRange\"", intensityrange, "list of numbers {start, end} or Undefined"],
		Block[{intentcode = intent /. $ReverseIntentCodes}, AnyTrue[MatchQ[intent, #] & /@ (Normal[$ReverseIntentCodes][[All, 1]]), TrueQ] && If[(2 <= intentcode[[1]] <= 21) && !MatchQ[intentcode[[2]], {Repeated[0., {1, 3}]}], ArrayDepth[data] == 5 && data[[5]] == 1 + Length[intentcode[[2]]], True]] /. False :> Message[Export::nptg, "\"Intent\"", intent, "member of " <> ToString[Normal[$ReverseIntentCodes][[All, 1]] /. {head_[Verbatim[Diffusica`Convert`NIfTIDump`p1_]] :> head["p1"], head_[Verbatim[Diffusica`Convert`NIfTIDump`p1_], Verbatim[Diffusica`Convert`NIfTIDump`p2_]] :> head["p1", "p2"], head_[Verbatim[Diffusica`Convert`NIfTIDump`p1_], Verbatim[Diffusica`Convert`NIfTIDump`p2_], Verbatim[Diffusica`Convert`NIfTIDump`p3_]] :> head["p1", "p2", "p3"]}] <> ", with p1, p2, and p3 real numbers"],
		(StringQ[intentname] && StringLength[intentname] <= 16) || intentname === Undefined /. False :> Message[Export::nptg, "\"IntentName\"", intentname, "string of 16 or fewer characters or Undefined"],
		(*Element[sepheaderbool, Booleans] /. False :> Message[Export::opttf, "\"SeparateHeader\"", sepheaderbool]*)sepheaderbool === False /. False :> Message[Export::general, "Export to separate header and image files is not currently supported."],
		sliceduration === Undefined || Positive[sliceduration] /. False :> Message[Export::nptg, "\"SliceAcquisitionDuration\"", sliceduration, "positive number or Undefined"],
		sliceordering === Undefined || (OptionQ[sliceordering] && Sort[sliceordering][[All, 1]] === {"FrequencyEncoding", "PhaseEncoding", "SliceEncoding"} && Max[Count[{"FrequencyEncoding", "PhaseEncoding", "SliceEncoding"} /. sliceordering, Normal[$ReverseDirections][[2 ;;, 1]]]] <= 1) /. False :> Message[Export::nptg, "\"SliceOrdering\"", sliceordering, "list of rules specifying the slice ordering"],
		slicerange === Undefined || (MatchQ[slicerange, {start_Integer ? NonNegative, end_Integer ? Positive} /; end > start]) /. False :> Message[Export::nptg, "\"SliceRange\"", slicerange, "list of integers {start, end} with end larger than start and start larger than zero, or Undefined"],
		ListQ[spatialtransformation] && Block[{len1 = Length[FilterRules[spatialtransformation, {"ArbitraryCoordinates", "ScannerBasedAnatomicalCoordinates"}]], len2 = Length[FilterRules[spatialtransformation, {"CoregisteredCoordinates", "TalairachTournouxAtlasAlignedCoordinates", "MNI152NormalizedCoordinates"}]]}, len1 <= 1 && len2 <= 1 && len1 + len2 == Length[spatialtransformation]] && AllTrue[spatialtransformation, MatchQ[#, ("ArbitraryCoordinates" -> _?ScalingTransformationQ) | (("ScannerBasedAnatomicalCoordinates" | "CoregisteredCoordinates" | "TalairachTournouxAtlasAlignedCoordinates" | "MNI152NormalizedCoordinates") -> _?AffineTransformationQ)] &] /. False :> Message[Export::nptg, "\"SpatialTransformation\"", spatialtransformation, "a list of rules identifying four dimensional affine TransformationFunction, with \"ArbitraryCoordinates\" restricted to a scaling transformation"], 
		MemberQ[Normal[$ReverseUnits][[All, 1]], spatialunit] /. False :> Message[Export::nptg, "\"SpatialUnit\"", spatialunit, "member of "<>ToString[Normal[$ReverseUnits][[All, 1]]]],
		MatchQ[temporaltransformation, TransformationFunction[{{_?NumberQ, _?NumberQ}, {0 | 0., 1 | 1.}}]] /. False :> Message[Export::nptg, "\"TemporalTransformation\"", temporaltransformation, "two dimensional TransformationFunction"], 
		MemberQ[Normal[$ReverseUnits][[All, 1]], temporalunit] /. False :> Message[Export::nptg, "\"SliceOrdering\"", temporalunit, "member of "<>ToString[Normal[$ReverseUnits][[All, 1]]]],
		MemberQ[Normal[$ReverseTimingOrder][[All, 1]], $TimingOrder] /. False :> Message[Export::nptg, "\"SliceOrdering\"", $TimingOrder, "member of "<>ToString[Normal[$ReverseTimingOrder][[All, 1]]]]
	}, Null], Return[$Failed, Module]];	
	
	(*Set Automatic header options*)
	datatype = datatype /. Automatic :> DetectDataType[Flatten[data]];
	If[datatype == {"Byte", "Byte", "Byte"}, ndim--];
	
	(*Check data type*)
	If[!ArrayQ[data, _, datatype /. $TypeChecks], 
		
		If[!ArrayQ[IntegerChop[data], _, datatype /. $TypeChecks],
			
			Message[Export::errstruct, "\"Data\". The type of the specified data is incompatible with the specified \"DataType\"."]; Return[$Failed, Module]
			
		]
		
	];
	
	(*Check data range*)
	Block[{rangecheck = datatype /. $RangeChecks},
			
		If[ListQ[rangecheck] && (Min[data] < rangecheck[[1]] || Max[data] > rangecheck[[2]]),
			
			Message[Export::errstruct, "\"Data\". The range of the specified data is incompatible with the specified \"DataType\""]; Return[$Failed, Module]
		
		]
		
	];	
	
	(*Check data dimensions *)
	If[datatype == {"Byte", "Byte", "Byte"} && Last[dim] != 3, Message[Export::nptg, "DataType", datatype, "token compatible with the dimensions of the input data"]; Return[$Failed, Module]];
	
	
	(*Intent checks*)
	If[
		!Switch[
			
			intent,
			
			{"Matrix", {_, _}}, ndim == 5 && (Times @@ intent[[2]]) == dim[[1]],
			{"SymmetricMatrix", {_, _}}, ndim == 5 && (intent[[2, 1]] (intent[[2, 1]] + 1) / 2) == dim[[1]],
			"DisplacementVector", ndim == 5,
			"Vector", ndim == 5,
			"Pointset", ndim == 5 && dim[[2]] == dim[[3]] == dim[[4]] == 1 && Positive[dim[[1]]],
			"Triangle", ndim == 5 && dim[[2]] == dim[[3]] == dim[[4]] == 1 && dim[[1]] == 3,
			"Quaternion", ndim == 5 && dim[[1]] == 4,
			_, True
				
		],
		
		Message[Export::nptg, "Intent", intent, "token compatible with the dimensions of the input data"];
		Return[$Failed, Module]
		
	];
	
	(*Spatial transformation check*)
	(*If[duplicate && Length[spatialtransformation] == 2 && spatialtransformation[[1]] =!= spatialtransformation[[2]], Message[Export::general, "Warning: the option value True for the option \"DuplicateSpatialTransformation\" will cause the secondary spatial transformation to be ignored."]];*)
		
	(*Return header options*)
	NIfTIExtendedCheckOptions[data, {
		"AuxiliaryFileName" 				-> aux, 
		"ByteOrdering" 						-> byteordering,
		"Compatibility"						-> compatibility, 
 		"DataScaling" 						-> datascaling, 
 		"DataType" 							-> datatype, 
 		"Description" 						-> description, 
 		"ExtendedHeader"					-> extheaderid,
 		"IntensityRange"					-> intensityrange, 
 		"Intent" 							-> intent, 
 		"IntentName" 						-> intentname, 
 		"Version" 							-> version, 
 		"NumberOfDimensions" 				-> ndim, 
 		"DataOffset" 						-> If[sepheaderbool, 0, Switch[version, 1, 352, 2, 544]], 
 		"SeparateHeader" 					-> sepheaderbool, 
 		"SliceAcquisitionDuration" 			-> sliceduration, 
 		"SliceOrdering" 					-> sliceordering, 
 		"SliceRange" 						-> slicerange, 
 		"SpatialTransformation" 			-> spatialtransformation, 
 		"SpatialUnit" 						-> spatialunit, 
 		"TemporalTransformation" 			-> temporaltransformation, 
 		"TemporalUnit" 						-> temporalunit, 
 		"TimingOrder" 						-> $TimingOrder
 	}, extheaderid, opts]
	
];

NIfTICheckOptions[_, _, version_, OptionsPattern[]] := (Message[Export::nptg, "\"Version\"", version, "positive integer identifying a supported NIfTI version"]; $Failed);

NIfTIExtendedCheckOptions[_, header_, Undefined, OptionsPattern[NIfTIDefaultExport]] := Module[
	{
		ext 			= OptionValue["Extensions"],
		offset 			= "DataOffset" /. header,
		sepheaderbool 	= "SeparateHeader" /. header,
		
		bytecount = {
			{_, string_String} :> (StringLength[string] + 8)
		}
	},

	If[ext == {},
		
		(*Return extended header format with modified header fields*)
		Sort[Join[{"DataOffset" -> If[sepheaderbool, 0, offset], "ExtendedHeader" -> None}, FilterRules[header, Except["DataOffset", "ExtendedHeader"]], {"Extensions" -> {}}]],
	
		(*Basic check*)
		If[!(ArrayQ[ext, depth_ /; depth >= 2] && Dimensions[ext][[2]] == 2 && And @@ (MatchQ[#, Alternatives @@ Normal[$ReverseECodes][[All, 1]]] & /@ ext[[All, 1]])),  
			Message[Export::nptg, "\"Extensions\"", ext, "list of rules describing valid header extensions, with lefthandsides members of " <> ToString[Normal[$ReverseECodes][[All, 1]] /. _Undefined -> Undefined["code"]] <> " where code is a positive integer."];
			Return[$Failed, Module]
		];
	
		(*Unparse*)
		ext[[All, 2]] = Unparse[ext];
	
		(*String check*)
		If[!MatchQ[ext[[All, 2]], {_String ...}], 
			Message[Export::nptg, "\"Extensions\"", ext, "list of rules describing valid header extensions, with lefthandsides members of " <> ToString[Normal[$ReverseECodes][[All, 1]]]/. _Undefined -> Undefined["code"] <> " where code is a positive integer."];
			Return[$Failed, Module]
		];

		(*Return extended header format with modified header fields*)
		Sort[Join[{"DataOffset" -> If[sepheaderbool, 0, offset + Total[ext /. bytecount]], "ExtendedHeader" -> Undefined}, FilterRules[header, Except["DataOffset", "ExtendedHeader"]], {"Extensions" -> ext}]]

	]	
	
];

NIfTIExtendedCheckOptions[data_, header_, "MiND", opts : OptionsPattern[NIfTIDefaultExport]] := Module[{
	
		extheader 		= "MiND" /. OptionValue["Extensions"],
		ndim 			= "NumberOfDimensions" /. header,
		intent 			= "Intent" /. header,
		offset 			= "DataOffset" /. header,
		sepheaderbool 	= "SeparateHeader" /. header,
		
		rules = {
			{"RAWDWI", list_} :> VectorQ[list, MatchQ[#[[2]], {_Real, _Real, _Real}] && CompatibleUnitQ[#[[1]], Quantity["Seconds"/("Millimeters"^2)]] || QuantityUnit[#[[1]]] === "DimensionlessUnit" &],
			{"DTENSOR", list_} :> (Sort[list] === {{1, 1}, {1, 2}, {1, 3}, {2, 2}, {2, 3}, {3, 3}}),
			{"DISCSPHFUNC", list_} :> MatchQ[list, {{_Real, _Real, _Real} ...}],
			{"REALSPHARMCOEFFS", list_} :> VectorQ[list, MatchQ[#, {l_Integer, m_Integer} /; -l <= m <= l] &]
		},
		bytecount = {
			{"RAWDWI", list_} :> (16 + 32 Length[list]),
			{"DTENSOR", list_} :> (16 + 16 Length[list]),
			{"DISCSPHFUNC", list_} :> (32 + 16 Length[list]),
			{"REALSPHARMCOEFFS", list_} :> (32 + 16 Length[list])
		},
		
		dim = Dimensions[data]
	
	},
	
	(*Check format*)
	If[
		Nand[
			VectorQ[extheader /. rules, TrueQ], 
			ndim == 5 == Length[dim], 
			Total[Length /@ extheader[[All, 2]]] == First[dim],
			intent === Undefined || intent == "Vector"
		],
		Message[Export::nptg, "\"Extensions\" (\"MiND\")", extheader, "valid list of MiND specifiers that are compatible with the dimensions of the data."];
		Return[$Failed, Module]
	];
	
	(*Return extended header format with modified header fields*)
	Sort[Join[{"DataOffset" -> If[sepheaderbool, 0, offset + Total[extheader /. bytecount]], "IntentName" -> "MiND"}, FilterRules[header, Except["DataOffset" | "IntentName"]], {"Extensions" -> OptionValue["Extensions"]}]]
	
];

NIfTIWriteHeader[stream_, data_, header_, 1, opts : OptionsPattern[]] := Module[{

		(*Header structure*)
		headertypes = {{"Integer32", 1}, {"Character8", 10}, {"Character8", 18}, {"Integer32", 1}, {"Integer16", 1}, {"Character8", 1}, {"Byte", 1}, {"Integer16", 8}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Integer16", 1}, {"Integer16", 1}, {"Integer16", 1}, {"Integer16", 1}, {"Real32", 8}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Integer16", 1}, {"Byte", 1}, {"Byte", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Integer32", 1}, {"Integer32", 1}, {"Character8", 80}, {"Character8", 24}, {"Integer16", 1}, {"Integer16", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 1}, {"Real32", 4}, {"Real32", 4}, {"Real32", 4}, {"Character8", 16}, {"Character8", 4}, {"Byte", 4}},
		
		(*Options*)
		extheaderid 			= "ExtendedHeader" /. header,
		byteordering 			= "ByteOrdering" /. header,
		offset 					= "DataOffset" /. header,
		unused					= ("Compatibility" /. header /. $CompatibilityModes)[data],
		sliceordering 			= {"FrequencyEncoding", "PhaseEncoding", "SliceEncoding"} /. ("SliceOrdering" /. header /. Undefined -> {"FrequencyEncoding" -> 0, "PhaseEncoding" -> 0, "SliceEncoding" -> 0}) /. $ReverseDirections,
		dimensions 				= PadRight[Join[{"NumberOfDimensions" /. header}, Reverse@Dimensions[data]], 8],
		intent 					= "Intent" /. header /. $ReverseIntentCodes,
		intensityrange 			= "IntensityRange" /. header /. Undefined -> {0., 0.},
		datatype 				= "DataType" /. header /. $ReverseDataTypes,
		slicerange 				= "SliceRange" /. header /. Undefined -> {0, 0},
		scaling 				= "DataScaling" /. header /. ((op : (Plus | Minus))[a_ ? NumberQ #, b_ ? NumberQ] &) :> {a, If[op == Minus, -b, b]} /. Identity -> {1., 0.}, 
		$TimingOrder 			= "TimingOrder" /. header /. $ReverseTimingOrder,
		spatialunit 			= "SpatialUnit" /. header /. $ReverseUnits,
		temporalunit 			= "TemporalUnit" /. header /. $ReverseUnits,
		sliceduration 			= "SliceAcquisitionDuration" /. header /. Undefined -> 0.,
		temporaltransformation 	= Normal["TemporalTransformation" /. header],
		description 			= "Description" /. header /. Undefined -> "",
		aux 					= "AuxiliaryFileName" /. header /. Undefined -> "",
		spatialtransformation1 	= Block[{spatialtransformation = FilterRules["SpatialTransformation" /. header, {"ArbitraryCoordinates", "ScannerBasedAnatomicalCoordinates"}]}, If[spatialtransformation != {}, First[spatialtransformation], "ArbitraryCoordinates" ->  TransformationFunction[{{1., 0., 0., 0.}, {0., 1., 0., 0.}, {0., 0., 1., 0.}, {0., 0., 0., 1.}}]]],
		spatialtransformation2 	= Block[{spatialtransformation = FilterRules["SpatialTransformation" /. header, {"CoregisteredCoordinates", "TalairachTournouxAtlasAlignedCoordinates", "MNI152NormalizedCoordinates"}]}, If[spatialtransformation != {}, First[spatialtransformation], "ArbitraryCoordinates" -> TransformationFunction[{{0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 1.}}]]],
		intentname 				= "IntentName" /. header /. Undefined -> "",
		magic 					= If["SeparateHeader" /. header, "ni1", "n+1"],
				
		(*Internal variables*)
		det, rotationmatrix, quaternions, outputheader

	},
	
	(*Compute quaternions and q-fac (det)*)
	If[spatialtransformation1[[1]] == "ScannerBasedAnatomicalCoordinates",
		
		rotationmatrix = Transpose[Normalize /@ Transpose[spatialtransformation1[[2, 1, ;; 3, ;; 3]]]];
		det = Det[rotationmatrix];
		If[det == -1., rotationmatrix[[All, 3]] *= -1];
		quaternions = RotationMatrixToQuaternionVector[rotationmatrix],
		
		det = 0.;
		quaternions = {0., 0., 0.}
	
	];
	
	(*Duplicate spatial transformation*)
	(*If[duplicate, spatialtransformation2 = spatialtransformation1];*)
	
	(*Pad intent*)
	intent[[2]] = PadRight[intent[[2]], 3];		
	
	(*Define raw header*)
	outputheader = {
		348, 
		unused[[1]], 
		unused[[2]], 
		unused[[3]], 
		unused[[4]], 
		unused[[5]], 
		sliceordering, 
		dimensions, 
		intent[[2, 1]], intent[[2, 2]], intent[[2, 3]], intent[[1]], 
		datatype, datatype /. $TypeSizes /. Undefined -> 0, 
		slicerange[[1]], 
		{
			det, 
			Norm[spatialtransformation1[[2, 1, ;; 3, 1]]], 
			Norm[spatialtransformation1[[2, 1, ;; 3, 2]]], 
			Norm[spatialtransformation1[[2, 1, ;; 3, 3]]], 
			temporaltransformation[[1, 1, 1]], 
			0., 
			0., 
			0.
		}, 
		offset, 
		scaling[[1]], scaling[[2]], 
		slicerange[[2]], 
		$TimingOrder, 
		{spatialunit, temporalunit}, 
		intensityrange[[2]], 
		intensityrange[[1]], 
		sliceduration, 
		temporaltransformation[[1, 1, 2]], 
		unused[[6]], 
		unused[[7]], 
		description, 
		aux, 
		spatialtransformation1[[1]] /. $ReverseForms, 
		spatialtransformation2[[1]] /. $ReverseForms,
		quaternions[[1]], 
		quaternions[[2]], 
		quaternions[[3]],
		spatialtransformation1[[2, 1, 1, 4]], 
		spatialtransformation1[[2, 1, 2, 4]], 
		spatialtransformation1[[2, 1, 3, 4]], 
		spatialtransformation2[[2, 1, 1]], 
		spatialtransformation2[[2, 1, 2]], 
		spatialtransformation2[[2, 1, 3]], 
		intentname, 
		magic, 
		{If[extheaderid =!= None, 1, 0], 0, 0, 0}
	};

	(*Assemble subparsed elements*)
	outputheader[[7]] = FromDigits[Flatten[IntegerDigits[#, 2, 2] & /@ Reverse[outputheader[[7]]]], 2];
	outputheader[[22]] = FromDigits[Flatten[IntegerDigits[#, 2, 3] & /@ Reverse[outputheader[[22]]/{1, 8}]], 2];

	outputheader[[21]] = {outputheader[[21]]};
	outputheader = Thread[{outputheader, headertypes}] /. {{val_, {"Character8", len_Integer}} :> PadRight[Characters[val], len, FromCharacterCode[0]], {val_, {_, 1}} :> {val}, {val_, {_, _}} :> val};

	(*Write header*)
	BinaryWrite[stream, outputheader, ConstantArray[#[[1]], {#[[2]]}] & /@ headertypes, ByteOrdering -> byteordering];

	(*Write extended header*)
	NIfTIExtendedWriteHeader[stream, data, header, extheaderid, opts]

];

NIfTIWriteHeader[stream_, data_, header_, 2, opts : OptionsPattern[]] := Module[{

		(*Header structure*)
		headertypes = {{"Integer32", 1}, {"Character8", 8}, {"Integer16", 1}, {"Integer16", 1}, {"Integer64", 8}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 8}, {"Integer64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Integer64", 1}, {"Integer64", 1}, {"Character8", 80}, {"Character8", 24}, {"Integer32", 1}, {"Integer32", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 1}, {"Real64", 4}, {"Real64", 4}, {"Real64", 4}, {"Integer32", 1}, {"Integer32", 1}, {"Integer32", 1}, {"Character8", 16}, {"Byte", 1}, {"Character8", 15}, {"Byte", 4}},
		
		(*Options*)
		extheaderid 			= "ExtendedHeader" /. header,
		byteordering 			= "ByteOrdering" /. header,
		offset 					= "DataOffset" /. header,
		sliceordering 			= {"FrequencyEncoding", "PhaseEncoding", "SliceEncoding"} /. ("SliceOrdering" /. header /. Undefined -> {"FrequencyEncoding" -> 0, "PhaseEncoding" -> 0, "SliceEncoding" -> 0}) /. $ReverseDirections,
		dimensions 				= PadRight[Join[{"NumberOfDimensions" /. header}, Reverse@Dimensions[data]], 8],
		intent 					= "Intent" /. header /. $ReverseIntentCodes,
		intensityrange 			= "IntensityRange" /. header /. Undefined -> {0., 0.},
		datatype 				= "DataType" /. header /. $ReverseDataTypes,
		slicerange 				= "SliceRange" /. header /. Undefined -> {0, 0},
		scaling 				= "DataScaling" /. header /. ((op : (Plus | Minus))[a_ ? NumericQ #, b_ ? NumericQ] &) :> {a, If[op == Minus, -b, b]} /. Identity -> {1., 0.}, 
		timingorder 			= "TimingOrder" /. header /. $ReverseTimingOrder,
		spatialunit 			= "SpatialUnit" /. header /. $ReverseUnits,
		temporalunit 			= "TemporalUnit" /. header /. $ReverseUnits,
		sliceduration 			= "SliceAcquisitionDuration" /. header /. Undefined -> 0.,
		temporaltransformation 	= Normal["TemporalTransformation" /. header],
		description 			= "Description" /. header /. Undefined -> "",
		aux 					= "AuxiliaryFileName" /. header /. Undefined -> "",
		spatialtransformation1 	= Block[{spatialtransformation = FilterRules["SpatialTransformation" /. header, {"ArbitraryCoordinates", "ScannerBasedAnatomicalCoordinates"}]}, If[spatialtransformation != {}, First[spatialtransformation], "ArbitraryCoordinates" ->  TransformationFunction[{{1., 0., 0., 0.}, {0., 1., 0., 0.}, {0., 0., 1., 0.}, {0., 0., 0., 1.}}]]],
		spatialtransformation2 	= Block[{spatialtransformation = FilterRules["SpatialTransformation" /. header, {"CoregisteredCoordinates", "TalairachTournouxAtlasAlignedCoordinates", "MNI152NormalizedCoordinates"}]}, If[spatialtransformation != {}, First[spatialtransformation], "ArbitraryCoordinates" -> TransformationFunction[{{0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 1.}}]]],
		intentname 				= "IntentName" /. header /. Undefined -> "",
		magic 					= If["SeparateHeader" /. header, "ni2", "n+2"],
				
		(*Internal variables*)
		det, rotationmatrix, quaternions, outputheader

	},
	
	(*Compute quaternions and q-fac (det)*)
	If[spatialtransformation1[[1]] == "ScannerBasedAnatomicalCoordinates",
		
		rotationmatrix = Transpose[Normalize /@ Transpose[spatialtransformation1[[2, 1, ;; 3, ;; 3]]]];
		det = Det[rotationmatrix];
		If[det == -1., rotationmatrix[[All, 3]] *= -1];
		quaternions = RotationMatrixToQuaternionVector[rotationmatrix],
		
		det = 0.;
		quaternions = {0., 0., 0.}
	
	];
	
	intent[[2]] = PadRight[intent[[2]], 3];		

	outputheader = {
		540, 
		magic,
		datatype, datatype /. $TypeSizes /. Undefined -> 0, 
		dimensions, 
		intent[[2, 1]], intent[[2, 2]], intent[[2, 3]], 
		{
			det, 
			Norm[spatialtransformation1[[2, 1, ;; 3, 1]]], 
			Norm[spatialtransformation1[[2, 1, ;; 3, 2]]], 
			Norm[spatialtransformation1[[2, 1, ;; 3, 3]]], 
			temporaltransformation[[1, 1, 1]], 
			0., 
			0., 
			0.
		}, 
		offset, 
		scaling[[1]], scaling[[2]], 
		intensityrange[[2]], 
		intensityrange[[1]], 
		sliceduration, 
		temporaltransformation[[1, 1, 2]], 
		slicerange[[1]], 
		slicerange[[2]], 
		description, 
		aux, 
		spatialtransformation1[[1]] /. $ReverseForms, 
		spatialtransformation2[[1]] /. $ReverseForms,
		quaternions[[1]], 
		quaternions[[2]], 
		quaternions[[3]],
		spatialtransformation1[[2, 1, 1, 4]], 
		spatialtransformation1[[2, 1, 2, 4]], 
		spatialtransformation1[[2, 1, 3, 4]], 
		spatialtransformation2[[2, 1, 1]], 
		spatialtransformation2[[2, 1, 2]], 
		spatialtransformation2[[2, 1, 3]], 
		timingorder, 
		{spatialunit, temporalunit}, 
		intent[[1]], 
		intentname, 
		sliceordering, 
		"",
		{If[extheaderid =!= None, 1, 0], 0, 0, 0}
	};

	(*Assemble subparsed elements*)
	outputheader[[36]] = FromDigits[Flatten[IntegerDigits[#, 2, 2] & /@ Reverse[outputheader[[36]]]], 2];
	outputheader[[33]] = FromDigits[Flatten[IntegerDigits[#, 2, 3] & /@ Reverse[outputheader[[33]]/{1, 8}]], 2];
	outputheader = Thread[{outputheader, headertypes}] /. {{val_, {"Character8", len_Integer}} :> PadRight[Characters[val], len, FromCharacterCode[0]], {val_, {_, 1}} :> {val}, {val_, {_, _}} :> val};

	(*Write header*)
	BinaryWrite[stream, outputheader, ConstantArray[#[[1]], {#[[2]]}] & /@ headertypes, ByteOrdering -> byteordering];

	(*Write extended header*)
	NIfTIExtendedWriteHeader[stream, data, header, extheaderid, opts]

];

NIfTIExtendedWriteHeader[_, _, _, None, OptionsPattern[]] := Null;

NIfTIExtendedWriteHeader[stream_, data_, header_, Undefined, OptionsPattern[]] := Module[{
	
		(*Options*)
		byteordering 	= "ByteOrdering" /. header,
		ext 			= "Extensions" /. header,
		
		bytecount = {
			{_, string_String} :> (StringLength[string] + 8)
		}
		
	},
	
	(*Write extended header*)
	Function[{extheader},(*Check extheader count mod 16*)
		BinaryWrite[stream, {extheader /. bytecount, extheader[[1]] /. $ReverseECodes}, {"Integer32", "Integer32"}, ByteOrdering -> byteordering];
		BinaryWrite[stream, extheader[[2]], ByteOrdering -> byteordering];
	] /@ ext	
	
];

NIfTIExtendedWriteHeader[stream_, data_, header_, "MiND", OptionsPattern[]] := Module[{
	
		(*Structures*)
		ecodestructure = {
			{"RAWDWI", list_List} :> (Flatten[{{{16, 18}, PadRight[Characters["RAWDWI"], 8, FromCharacterCode[0]]}, Riffle[Flatten[list, 1], {{16, 20}, {16, 22}}, {1, -2, 2}]}, 1] /. {quantity_?QuantityQ :> QuantityMagnitude[quantity, "Seconds"/("Millimeters"^2)], vec : {_, _, _} :> CartesianToSphericalCoordinates[vec]}),
			{"DTENSOR", list_List} :> Flatten[{{{16, 18}, PadRight[Characters["DTENSOR"], 8, FromCharacterCode[0]]}, Riffle[list, {{16, 24}}, {1, -2, 2}]}, 1],
			{"DISCSPHFUNC", list_List} :> (Flatten[{{{32, 18}, PadRight[Characters["DISCSPHFUNC"], 24, FromCharacterCode[0]]}, Riffle[list, {{16, 22}}, {1, -2, 2}]}, 1] /. vec : {_, _, _} :> CartesianToSphericalCoordinates[vec]),
			{"REALSPHARMCOEFFS", list_List} :> Flatten[{{{32, 18}, PadRight[Characters["REALSPHARMCOEFFS"], 24, FromCharacterCode[0]]}, Riffle[list, {{16, 26}}, {1, -2, 2}]}, 1]
		},
		typestructure = {
			{16,18} -> {"Integer32", "Integer32", ConstantArray["Character8", {8}]},
			{32,18} -> {"Integer32", "Integer32", ConstantArray["Character8", {24}]},
			{16,20} -> {"Integer32", "Integer32", "Real64"},
			{16,22} -> {"Integer32", "Integer32", "Real32", "Real32"},
			{16,24} -> ConstantArray["Integer32", {4}],
			{16,26} -> ConstantArray["Integer32", {4}]
		},
		
		(*Options*)
		byteordering = "ByteOrdering" /. header,
		mind = "MiND" /. ("Extensions" /. header),
		
		(*Internal variables*)
		extheader, headertypes
		
	},
	
	(*Reassemble header*)
	extheader = Flatten[mind /. ecodestructure, 1];
	headertypes = Flatten[extheader[[1 ;; -2 ;; 2]] /. typestructure];
		
	(*Write MiND header*)
	BinaryWrite[stream, Flatten[extheader], headertypes, ByteOrdering -> byteordering];
	
];
	
NIfTIWriteData[stream_, data_, header_, OptionsPattern[]] := Module[{

		(*Options*)
		byteordering = "ByteOrdering" /. header,
		datatype = "DataType" /. header,
		reversescaling = InverseFunction["DataScaling" /. header]
	
	},
	
	(*Write data*)
	Check[BinaryWrite[stream, reversescaling @ Flatten[data], datatype, ByteOrdering -> byteordering], Return[$Failed, Module]]	
	
];

End[];
