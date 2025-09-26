#ifndef TABU_SEARCH_HPP
#define TABU_SEARCH_HPP

#include "../sc-qbf/sc_qbf.hpp"
#include <vector>
#include <random>
#include <set>
#include <deque>
#include <unordered_set>
#include <functional>

using namespace std;

class TabuSearch {
public:
    enum SearchMethod {
        FIRST_IMPROVING,
        BEST_IMPROVING
    };

    enum TabuStrategy {
        STANDARD,
        INTENSIFICATION_RESTART
    };

    struct Move {
        int variable;       // Variable index
        int from_value;     // Previous value (0 or 1)
        int to_value;       // New value (0 or 1)
        double delta;       // Change in objective function value

        Move() : variable(-1), from_value(0), to_value(0), delta(0.0) {}
        Move(int var, int from, int to, double d) : variable(var), from_value(from), to_value(to), delta(d) {}

        bool operator==(const Move& other) const {
            return variable == other.variable && from_value == other.from_value && to_value == other.to_value;
        }
    };

    // Hash function for Move to be used in unordered_set
    struct MoveHash {
        size_t operator()(const Move& move) const {
            return hash<int>()(move.variable) ^
                (hash<int>()(move.from_value) << 1) ^
                (hash<int>()(move.to_value) << 2);
        }
    };

private:
    // Algorithm parameters
    int tabuTenure;                    // Size of the tabu list
    int maxIterations;                 // Maximum number of iterations
    int timeLimit;                     // Time limit in seconds
    SearchMethod searchMethod;
    TabuStrategy tabuStrategy;

    // Algorithm state
    deque<Move> tabuList;                  // Tabu list implemented as deque
    unordered_set<Move, MoveHash> tabuSet; // For fast lookup in tabu list
    vector<int> bestSolution;              // Best solution found
    double bestValue;                      // Best value found
    vector<int> currentSolution;           // Current solution
    double currentValue;                   // Current value

    // Counters and history
    int iterationsWithoutImprovement;
    int totalIterations;
    vector<double> valueHistory;       // History of values for analysis
    vector<int> frequencyMatrix;       // Frequency matrix for diversification

    mutable mt19937 rng;               // Random number generator

private:
    void applyIntensificationRestart(const SetCoverQBF& scqbf);
    
public:
    TabuSearch();
    TabuSearch(int tenure, int maxIter, int timeLimit,
        SearchMethod sm = FIRST_IMPROVING,
        TabuStrategy ts = STANDARD);

    // Main method
    vector<int> run(const SetCoverQBF& scqbf);

    // Configuration setters
    void setTabuTenure(int tenure) { tabuTenure = tenure; }
    void setMaxIterations(int maxIter) { maxIterations = maxIter; }
    void setTimeLimit(int timeLimit) { this->timeLimit = timeLimit; }
    void setSearchMethod(SearchMethod sm) { searchMethod = sm; }
    void setTabuStrategy(TabuStrategy ts) { tabuStrategy = ts; }

    // Getters
    int getTabuTenure() const { return tabuTenure; }
    int getMaxIterations() const { return maxIterations; }
    int getTimeLimit() const { return timeLimit; }
    SearchMethod getSearchMethod() const { return searchMethod; }
    TabuStrategy getTabuStrategy() const { return tabuStrategy; }
    double getBestValue() const { return bestValue; }
    int getTotalIterations() const { return totalIterations; }

    // Auxiliary getters
    vector<double> getValueHistory() const { return valueHistory; }
private:
    // Initialization methods
    void initialize(const SetCoverQBF& scqbf);
    vector<int> generateInitialSolution(const SetCoverQBF& scqbf);
    vector<int> generateGreedySolution(const SetCoverQBF& scqbf);
    vector<int> generateRandomSolution(const SetCoverQBF& scqbf);

    // Main TS methods
    bool iterate(const SetCoverQBF& scqbf);
    vector<Move> generateNeighborhood(const SetCoverQBF& scqbf, const vector<int>& solution);
    Move selectMove(const SetCoverQBF& scqbf, const vector<Move>& moves);

    // Search methods
    Move selectFirstImproving(const SetCoverQBF& scqbf, const vector<Move>& moves);
    Move selectBestImproving(const SetCoverQBF& scqbf, const vector<Move>& moves);

    // Tabu list management
    bool isTabu(const Move& move) const;
    void addToTabuList(const Move& move);

    // Aspiration criterion
    bool aspirationCriterion(const Move& move, double newValue) const;

    // Alternative tabu strategies
    void applyTabuStrategy(const SetCoverQBF& scqbf);

    // Auxiliary methods
    double calculateMoveDelta(const SetCoverQBF& scqbf, const vector<int>& solution, const Move& move) const;
    void applyMove(vector<int>& solution, const Move& move);
    void updateBestSolution(const vector<int>& solution, double value);
    void updateFrequencyMatrix(const vector<int>& solution);

    // Repair methods to ensure feasibility
    vector<int> repairSolution(const SetCoverQBF& scqbf, const vector<int>& solution);
    bool needsRepair(const SetCoverQBF& scqbf, const vector<int>& solution) const;
};

#endif
