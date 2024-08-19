// SolverTest.cpp : Defines the entry point for the application.
//

#include "SolverTest.h"

#include <BasicTests.h>
#include <NQueens.h>
#include <EATest/EATest.h>
#include <Sudoku.h>
#include <TowersOfHanoi.h>
#include <PrefabTest.h>
#include <TileTests.h>
#include <ZoneGraph.h>

#include "KnightTourSolver.h"
#include "ds/ValueBitset.h"
#include "Maze.h"

using namespace Vertexy;

static constexpr int FORCE_SEED = 0; //391127821 & 1368894268 (recent failure 3/13) //1618771936 (recent failure 8/13) // 1890479072 (recent success 13/13)
static constexpr int NUM_TIMES = 1;
static constexpr int MAZE_NUM_ROWS = 15;
static constexpr int MAZE_NUM_COLS = 15;
static constexpr int NQUEENS_SIZE = 25;
static constexpr int SUDOKU_STARTING_HINTS = 0;
static constexpr int KNIGHT_BOARD_DIM = 6;
static constexpr bool PRINT_VERBOSE = true;

int main(int argc, char* argv[])
{
	using namespace EA::UnitTest;
	using namespace VertexyTests;

	TestApplication Suite("Solver Tests", argc, argv);

	/*
	Suite.AddTest("ValueBitset", TestSolvers::bitsetTests);
	Suite.AddTest("Digraph", TestSolvers::digraphTests);
	Suite.AddTest("RuleSCCs", TestSolvers::ruleSCCTests);
	Suite.AddTest("Clause-Basic", []() { return TestSolvers::solveClauseBasic(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Inequality-Basic", []() { return TestSolvers::solveInequalityBasic(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Cardinality-Basic", []() { return TestSolvers::solveCardinalityBasic(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Cardinality-Shift", []() { return TestSolvers::solveCardinalityShiftProblem(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("AllDifferent-Small", []() { return TestSolvers::solveAllDifferentSmall(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("AllDifferent-Large", []() { return TestSolvers::solveAllDifferentLarge(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Rules-BasicChoice", []() { return TestSolvers::solveRules_basicChoice(FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Rules-BasicDisjunction", []() { return TestSolvers::solveRules_basicDisjunction(FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Rules-BasicCycle", []() { return TestSolvers::solveRules_basicCycle(FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Rules-BasicGraph", []() { return TestSolvers::solveProgram_graphTests(FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Rules-Hamiltonian", []() { return TestSolvers::solveProgram_hamiltonian(FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Rules-HamiltonianGraph", []() { return TestSolvers::solveProgram_hamiltonianGraph(FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Sum-Basic", []() { return TestSolvers::solveSumBasic(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Sudoku", []() { return SudokuSolver::solve(NUM_TIMES, SUDOKU_STARTING_HINTS, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("TowersOfHanoi", []() { return TowersOfHanoiSolver::solve(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("KnightTour", []() { return KnightTourSolver::solve(NUM_TIMES, KNIGHT_BOARD_DIM, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("NQueens-AllDifferent", []() { return NQueensSolvers::solveUsingAllDifferent(NUM_TIMES, NQUEENS_SIZE, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("NQueens-Table", []() { return NQueensSolvers::solveUsingTable(NUM_TIMES, NQUEENS_SIZE, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("NQueens-Graph", []() { return NQueensSolvers::solveUsingGraph(NUM_TIMES, NQUEENS_SIZE, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("PrefabTest-Basic", []() { return PrefabTestSolver::solveBasic(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("PrefabTest-Json", []() { return PrefabTestSolver::solveJson(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("PrefabTest-Neighbor", []() { return PrefabTestSolver::solveNeighbor(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("PrefabTest-Rot/Refl", []() { return PrefabTestSolver::solveRotationReflection(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("MazeProgram", []() { return MazeSolver::solveUsingGraphProgram(NUM_TIMES, MAZE_NUM_ROWS, MAZE_NUM_COLS, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("Maze", []() { return MazeSolver::solveUsingRawConstraints(NUM_TIMES, MAZE_NUM_ROWS, MAZE_NUM_COLS, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("TileTest-Basic", []() { return TileTests::solveBasic(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	Suite.AddTest("TileTest-Rot/Ref", []() { return TileTests::solveRotationReflection(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	*/

	//Suite.AddTest("ShortestPath-Max-1", []() { return TestSolvers::solveShortestPath(NUM_TIMES, FORCE_SEED, make_tuple(0, 1), 6, false, 10, PRINT_VERBOSE); });
	//Suite.AddTest("ShortestPath-Max-2", []() { return TestSolvers::solveShortestPath(NUM_TIMES, FORCE_SEED, make_tuple(0, 2), 6, false, 18, PRINT_VERBOSE); });
	//Suite.AddTest("ShortestPath-Max-3", []() { return TestSolvers::solveShortestPath(NUM_TIMES, FORCE_SEED, make_tuple(0, 3), 6, false, 24, PRINT_VERBOSE); });
	//Suite.AddTest("ShortestPath-Max-4", []() { return TestSolvers::solveShortestPath(NUM_TIMES, FORCE_SEED, make_tuple(0, 4), 6, false, 28, PRINT_VERBOSE); });
	//Suite.AddTest("ShortestPath-Max-5", []() { return TestSolvers::solveShortestPath(NUM_TIMES, FORCE_SEED, make_tuple(0, 5), 6, false, 30, PRINT_VERBOSE); });
	//Suite.AddTest("ShortestPath-ReachabilityOnly", []() { return TestSolvers::solveShortestPath(NUM_TIMES, FORCE_SEED, make_tuple(0, INT_MAX-1), 6, false, 30, PRINT_VERBOSE); });
	Suite.AddTest("ShortestPath-Max-All-2", []() { return TestSolvers::solveShortestPath(NUM_TIMES, FORCE_SEED, make_tuple(0, 2), 6, true, 13, PRINT_VERBOSE); });
	//Suite.AddTest("ShortestPath-Min", []() { return TestSolvers::solveShortestPath(NUM_TIMES, FORCE_SEED, make_tuple(5, INT_MAX-1), 6, true, 2, true); });
	//Suite.AddTest("ShortestPath-Range", []() { return TestSolvers::solveShortestPath_Range(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	//Suite.AddTest("Cardinality-Shift", []() { return TestSolvers::solveCardinalityShiftProblem(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	//Suite.AddTest("ZoneGraph", []() { return ZoneGraphSolver::solve(NUM_TIMES, 50, 4, FORCE_SEED, PRINT_VERBOSE); });
	//Suite.AddTest("Cardinality-Basic", []() { return TestSolvers::solveCardinalityBasic(NUM_TIMES, FORCE_SEED, PRINT_VERBOSE); });
	//Suite.AddTest("MazeProgram", []() { return MazeSolver::solveUsingGraphProgram(NUM_TIMES, MAZE_NUM_ROWS, MAZE_NUM_COLS, FORCE_SEED, PRINT_VERBOSE); });
	//Suite.AddTest("Maze", []() { return MazeSolver::solveUsingRawConstraints(NUM_TIMES, MAZE_NUM_ROWS, MAZE_NUM_COLS, FORCE_SEED, PRINT_VERBOSE); });
	//Suite.AddTest("Maze", []() { return ZoneGraphSolver::solveUsingGraphProgram(NUM_TIMES, 5, 10, FORCE_SEED, PRINT_VERBOSE); });

	return Suite.Run();
}