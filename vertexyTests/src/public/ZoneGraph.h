// Copyright Norm Nazaroff. All Rights Reserved.

#pragma once

#include "ConstraintTypes.h"
#include "topology/TopologyVertexData.h"
#include "topology/DigraphTopology.h"

namespace VertexyTests
{

	using namespace Vertexy;
	class ZoneGraphSolver
	{
		ZoneGraphSolver()
		{
		}

	public:
		static void write(const wchar_t* outputFile, const shared_ptr<DigraphTopology>& graph);
		static int solveUsingGraphProgram(int times, int numZones, int seed, bool printVerbose = true);
		static int solve(int times, int numZones, int maxConnections, int seed, bool printVerbose = true);
		//static int check(const shared_ptr<TTopologyVertexData<VarID>>& Cells, const ConstraintSolver& solver);
		//static void print(const shared_ptr<TTopologyVertexData<VarID>>& cells, const shared_ptr<TTopologyVertexData<VarID>>& edges, const ConstraintSolver& solver);
	};
}
