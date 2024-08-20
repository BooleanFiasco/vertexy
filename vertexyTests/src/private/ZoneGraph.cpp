// Copyright Norm nazaroff. All Rights Reserved.

#include "ZoneGraph.h"

#include "ConstraintSolver.h"
#include "EATest/EATest.h"
#include "constraints/ReachabilityConstraint.h"
#include "constraints/PathDistanceConstraint.h"
#include "constraints/ShortestPathConstraint.h"
#include "constraints/TableConstraint.h"
#include "constraints/IffConstraint.h"
#include "decision/LogOrderHeuristic.h"
#include "program/ProgramDSL.h"
#include "topology/GridTopology.h"
#include "topology/GraphRelations.h"
#include "topology/IPlanarTopology.h"
#include "util/SolverDecisionLog.h"
#include "variable/SolverVariableDomain.h"

#include <iostream>
#include <fstream>
#include <string>
#include <random>

using namespace VertexyTests;

// Interval of solver steps to print out current maze status. Set to 1 to see each solver step.
static constexpr int MAZE_REFRESH_RATE = 0;
// The number of keys/doors that should exist in the maze.
static constexpr int NUM_KEYS = 2;
// True to print edge variables in FMaze::Print
static constexpr bool PRINT_EDGES = false;
// Whether to write a decision log as DecisionLog.txt
static constexpr bool WRITE_BREADCRUMB_LOG = true;
// Whether to write a solution file after a solution is found
static constexpr bool WRITE_SOLUTION_FILE = false;
static constexpr bool WRITE_OUTPUT_FILE = true;
// If >= 0, the step during solving to read and attempt to apply a previously-written solution file.
// Useful for debugging constraints.
static constexpr int ATTEMPT_SOLUTION_AT = -1;

// This implements this decision heuristic interface, but just forwards the decision to the default heuristic.
// Its only purpose is to print out the maze whenever a conflict is detected, for debugging purposes.
/*
class DebugMazeStrategy : public ISolverDecisionHeuristic
{
public:
	DebugMazeStrategy(ConstraintSolver& solver, const shared_ptr<TTopologyVertexData<VarID>>& cells, const shared_ptr<TTopologyVertexData<VarID>>& edges)
		: m_cells(cells)
		, m_edges(edges)
		, m_solver(solver)
	{
	}

	virtual bool getNextDecision(SolverDecisionLevel level, VarID& var, ValueSet& chosenValues) override
	{
		// defer to default
		return false;
	}

	virtual void onClauseLearned() override
	{
		MazeSolver::print(m_cells, m_edges, m_solver);
	}

protected:
	shared_ptr<TTopologyVertexData<VarID>> m_cells;
	shared_ptr<TTopologyVertexData<VarID>> m_edges;
	ConstraintSolver& m_solver;
};
*/

void ZoneGraphSolver::write(const wchar_t* outputFile, const shared_ptr<DigraphTopology>& graph)
{
	
}

int ZoneGraphSolver::solve(int times, int numZones, int maxConnections, int seed, bool printVerbose)
{
	int nErrorCount = 0;

	// Need this to actually do interesting things!
	ConstraintSolver solver(TEXT("ZoneGraph"), seed);

	// Set up the graph of zones
	shared_ptr<DigraphTopology> zoneGraph = make_shared<DigraphTopology>();

	// Create all the zone vertices
	vector<int> nodes = {};
	for (int i = 0; i < numZones; ++i)
	{
		nodes.push_back(zoneGraph->addVertex());
	}

	VERTEXY_LOG("  CREATED %d ZONES...", nodes.size());

	//SolverVariableDomain zoneTypeDomain(0, 1);
	// Define all valid zone types
	VXY_DOMAIN_BEGIN(ZoneTypeDomain)
		VXY_DOMAIN_VALUE(combat);
		VXY_DOMAIN_VALUE(basecamp);
		VXY_DOMAIN_VALUE(boss);
		//VXY_DOMAIN_VALUE(oasis);
	VXY_DOMAIN_END()

	auto zoneTypeDomain = ZoneTypeDomain::get()->getSolverDomain();
	auto zoneTypeData = solver.makeVariableGraph(TEXT("ZoneData"), ITopology::adapt(zoneGraph), zoneTypeDomain, TEXT("type"));

	const vector<int> reachableZones{ ZoneTypeDomain::get()->combat.getValueIndex() }; // Any zone other than the basecamp
	const vector<int> originZone{ ZoneTypeDomain::get()->basecamp.getValueIndex() }; // Basecamp zone, all other zones need to be reachable from here
	const vector<int> spreadZones{ ZoneTypeDomain::get()->boss.getValueIndex() };

	// Arbitrarily use the first node as origin (it doesn't matter since all nodes are fully connected!)
	//solver.setInitialValues(zoneTypeData->get(nodes[0]), originZone);

	// Only one basecamp and boss zone is allowed!
	hash_map<int, tuple<int, int>> zoneTypeCardinalities;
	zoneTypeCardinalities[ZoneTypeDomain::get()->basecamp.getValueIndex()] = make_tuple(1, 1);
	zoneTypeCardinalities[ZoneTypeDomain::get()->boss.getValueIndex()] = make_tuple(1, 1);

	// Just make zone-0 the basecamp always
	solver.setInitialValues(zoneTypeData->get(0), vector{ 1 });

	// Just make zone-1 the boss always
	solver.setInitialValues(zoneTypeData->get(1), vector{ 2 });

	solver.cardinality(zoneTypeData->getData(), zoneTypeCardinalities);

	// Fully connect nodes
	int numDirectionalEdges = 0;
	for (int i = 0; i < nodes.size(); ++i)
	{
		for (int j = 0; j < nodes.size(); ++j)
		{
			if (i == j) continue;

			zoneGraph->addEdge(nodes[i], nodes[j]);
			numDirectionalEdges++;
		}
	}

	VERTEXY_LOG("  CREATED %d DIRECTIONAL EDGES...", numDirectionalEdges);

	// Create a graph of all edges so we can map them to variables
	constexpr bool mergeBidirectionalEdges = true; // TODO: Try disabling this to create more interesting graphs once the basics are working!
	auto edges = make_shared<EdgeTopology>(ITopology::adapt(zoneGraph), mergeBidirectionalEdges, false);

	const int numBidirectionalEdges = edges->getNumVertices();
	VERTEXY_LOG("  MERGED DOWN TO %d BI-DIRECTIONAL EDGES...", numBidirectionalEdges);

	// Edges can either be closed (0) or open (1)
	SolverVariableDomain pathDomain(0, 1);
	auto pathOpenData = solver.makeVariableGraph(TEXT("EdgeData"), ITopology::adapt(edges), pathDomain, TEXT("pathOpen"));

	const vector<int> closedPath{ 0 }; // Blocks reachability
	const vector<int> openPath{ 1 }; // Allows reachability

	const int numPathVars = pathOpenData->getData().size();
	VERTEXY_LOG("  CREATED %d PATH VARS...", numPathVars);

	// Restrict the maximum number of allowed connections. Note that we don't allow islands by making the min
	// value 1, but we don't actually have to do this since we also have a reachability constraint!
	hash_map<int, tuple<int, int>> pathOpenCardinalities;
	pathOpenCardinalities[1] = make_tuple(1, maxConnections);

	// Constrain number of open paths for every node
	for (auto node : nodes)
	{
		vector<VarID> edgeVars {};
		auto neighbors = zoneGraph->getNeighbors(node);
		for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
		{
			auto edgeVert = edges->getVertexForSourceEdge(node, *it);
			edgeVars.push_back(pathOpenData->get(edgeVert));
		}
		
		VERTEXY_LOG("  CONSTRAINING %d PATHS FOR ZONE %d...", edgeVars.size(), node);
		solver.cardinality(edgeVars, pathOpenCardinalities);
	}

	// Ensure reachability for this step: all Step_Reachable cells must be reachable from Step_Origin cells.
	tuple<int, int> distLimits = make_tuple(0, 5);
	solver.makeConstraint<ShortestPathConstraint>(zoneTypeData, originZone, reachableZones, pathOpenData, closedPath, distLimits);

	//tuple<int, int> bossDistLimits = make_tuple(3, INT_MAX);
	//solver.makeConstraint<PathDistanceConstraint>(zoneTypeData, originZone, spreadZones, pathOpenData, closedPath, bossDistLimits);

	VERTEXY_LOG("  READY TO SOLVE! STARTING...");

	//for (int i = 0; i < times; ++i)
	//{
		auto result = solver.solve();
		EATEST_VERIFY(result == EConstraintSolverResult::Solved);

		if (printVerbose && result == EConstraintSolverResult::Solved)
		{
			//VERTEXY_LOG("   SOLUTION %d:", i+1);
			VERTEXY_LOG("   SOLUTION:");
			for (auto it = zoneTypeData->getData().begin(); it != zoneTypeData->getData().end(); ++it)
			{
				const int value = solver.getSolvedValue(*it);
				if (value == 1)
				{
					VERTEXY_LOG("    %s is basecamp!", solver.getVariableName(*it).c_str());
				}
				else if (value == 2)
				{
					VERTEXY_LOG("    %s is boss!", solver.getVariableName(*it).c_str());
				}
			}

			for (auto it = pathOpenData->getData().begin(); it != pathOpenData->getData().end(); ++it)
			{
				const int value = solver.getSolvedValue(*it);
				if (value != 0)
				{
					VERTEXY_LOG("    %s = %d", solver.getVariableName(*it).c_str(), value);
				}
			}
		}

		solver.dumpStats(printVerbose);
	//}

	

	//
	// Write two output files: one for nodes, and the other for connections
	//
	if (result == EConstraintSolverResult::Solved)
	{
		// Nodes file
		std::basic_ofstream<wchar_t> file(TEXT("ZoneGraph-Nodes-Output.csv"));
		vxy_verify(file.is_open());

		file << TEXT("id,type\n");

		hash_map<int, wstring> zoneTypeNames;
		zoneTypeNames[0] = TEXT("combat");
		zoneTypeNames[1] = TEXT("basecamp");
		zoneTypeNames[2] = TEXT("boss");

		for (auto node : nodes)
		{
			auto nodeTypeVarId = zoneTypeData->get(node);
			const int value = solver.getSolvedValue(nodeTypeVarId);
			wstring line = { wstring::CtorSprintf(), TEXT("zone-%d,%s"), node, zoneTypeNames[value] };
			file << line.c_str() << TEXT("\n");
		}

		file.close();
	}
	
	if (result == EConstraintSolverResult::Solved)
	{
		// Connections file
		std::basic_ofstream<wchar_t> file(TEXT("ZoneGraph-Connections-Output.csv"));
		vxy_verify(file.is_open());

		file << TEXT("Source,Target\n");

		for (int i = 0; i < edges->getNumVertices(); ++i)
		{
			auto pathVar = pathOpenData->get(i);
			auto value = solver.getSolvedValue(pathVar);
			if (value != 0)
			{
				wstring line = { wstring::CtorSprintf(), TEXT("zone-%d,zone-%d"), edges->getVertices()[i].sourceFrom, edges->getVertices()[i].sourceTo };
				file << line.c_str() << TEXT("\n");
			}
		}

		file.close();
	}

	return nErrorCount;
}

int ZoneGraphSolver::solveUsingGraphProgram(int times, int numZones, int seed, bool printVerbose)
{
	int nErrorCount = 0;

	//
	// CREATE ZONE VERTICES
	//

	// Define all valid zone types
	VXY_DOMAIN_BEGIN(ZoneDomain)
		VXY_DOMAIN_VALUE(basecamp);
		VXY_DOMAIN_VALUE(empty);
		VXY_DOMAIN_VALUE(combat);
		VXY_DOMAIN_VALUE(boss);
	VXY_DOMAIN_END()

	// Declare a formula called "zoneType" of the given domain, with one argument.
	VXY_DOMAIN_FORMULA(zoneType, ZoneDomain, 1);

	// Set up the graph of zones
	shared_ptr<DigraphTopology> zoneGraph = make_shared<DigraphTopology>();

	// Create all the zone vertices
	vector<int> nodes = {};
	for (int i = 0; i < numZones; ++i)
	{
		nodes.push_back(zoneGraph->addVertex());
	}

	// Need this to actually do interesting things!
	ConstraintSolver solver(TEXT("ZoneGraph"), seed);
	
	// Allocate data to store zone type and bind it
	auto zoneDomain = ZoneDomain::get()->getSolverDomain();
	auto zoneTypeData = solver.makeVariableGraph(TEXT("ZoneData"), ITopology::adapt(zoneGraph), zoneDomain, TEXT("type"));
	zoneType.bind(solver, [&](const ProgramSymbol& vert)
	{
		return zoneTypeData->get(vert.getInt());
	});

	// For every graph vertex, set a random limit for how many neighbors it's allowed to have
	//constexpr int minConnections = 1;
	//constexpr int maxConnections = 5;
	//SolverVariableDomain connectionLimitDomain(minConnections, maxConnections);
	//auto connectionLimitData = solver.makeVariableGraph(TEXT("ZoneData"), IPlanarTopology::adapt(zoneGraph), connectionLimitDomain, TEXT("connectionLimit"));
	
	//
	// CREATE ZONE CONNECTIONS
	//

	// Fully connect nodes
	for (int i = 0; i < nodes.size() - 1; ++i)
	{
		for (int j = 0; j < nodes.size() - 1; ++j)
		{
			if (i == j) continue;

			zoneGraph->addEdge(nodes[i], nodes[j]);
		}
	}

	constexpr bool mergeBidirectionalEdges = true; // TODO: Try disabling this to create more interesting graphs once the basics are working!
	auto edges = make_shared<EdgeTopology>(ITopology::adapt(zoneGraph), mergeBidirectionalEdges, false);
	VXY_DOMAIN_BEGIN(EdgeDomain)
		VXY_DOMAIN_VALUE(closed);
		VXY_DOMAIN_VALUE(open);
	VXY_DOMAIN_END()

	auto edgeDomain = EdgeDomain::get()->getSolverDomain();
	auto edgeOpenData = solver.makeVariableGraph(TEXT("EdgeData"), ITopology::adapt(edges), edgeDomain, TEXT("edgeOpen"));

	VXY_DOMAIN_BEGIN(ZoneStepDomain)
		VXY_DOMAIN_VALUE(unreachable);
		VXY_DOMAIN_VALUE(reachable);
		VXY_DOMAIN_VALUE(origin);
	VXY_DOMAIN_END()

	struct Result
	{
		FormulaResult<2, ZoneStepDomain> zoneStep;
		FormulaResult<3> edgeOpen;
	};

	auto prg = Program::define([&](ProgramVertex vertex, int basecampVertex, int bossVertex)
	{
		// Designate one vertex as the basecamp
		zoneType(basecampVertex).is(zoneType.basecamp);

		// Designate one vertex as the basecamp
		zoneType(bossVertex).is(zoneType.boss);

		// Specify types that each node could be
		zoneType(vertex).is(zoneType.empty | zoneType.combat);

		// Now...somehow open connections between nodes such that:
		//	1. They can all path to the basecamp
		//	2. The boss zone is at least 3 zones away from the basecamp
		//	3. No zone has more than 5 neighbors (ideally this is randomized!)

		//stepEdgeData->getData()[vertex
	});

	// Write CSV that's compatible with Cytoscape to visualize the final graph
	std::basic_ofstream<wchar_t> file(TEXT("ZoneGraph-Output.csv"));
	vxy_verify(file.is_open());
	
	file << TEXT("Source,Target\n");

	for (int i = 0; i < nodes.size(); ++i)
	{
		int vertex = nodes[i];
		auto neighbors = zoneGraph->getNeighbors(vertex);
		for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
		{
			wstring line = { wstring::CtorSprintf(), TEXT("zone-%d,zone-%d"), vertex, *it };
			file << line.c_str() << TEXT("\n");
		}
	}

	file.close();

	return nErrorCount;
}
