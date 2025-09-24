#include <algorithm>
#include <chrono>
#include <iostream>
#include <climits>
#include <cmath>
#include "ts.hpp"

TabuSearch::TabuSearch() : tabuTenure(7), maxIterations(1000), timeLimit(1800),
searchMethod(FIRST_IMPROVING), tabuStrategy(STRATEGIC_OSCILLATION), bestValue(-1e9), currentValue(-1e9),
iterationsWithoutImprovement(0), totalIterations(0) {
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    rng.seed(seed);
}

TabuSearch::TabuSearch(int tenure, int maxIter, int timeLimit, SearchMethod sm, TabuStrategy ts)
    : tabuTenure(tenure), maxIterations(maxIter), timeLimit(timeLimit),
    searchMethod(sm), tabuStrategy(ts), bestValue(-1e9),
    currentValue(-1e9), iterationsWithoutImprovement(0), totalIterations(0) {
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    rng.seed(seed);
}

vector<int> TabuSearch::run(const SetCoverQBF& scqbf) {
    cout << "Starting Tabu Search for MAX-SC-QBF..." << endl;
    cout << "Parameters: tenure=" << tabuTenure << ", maxIter=" << maxIterations
        << ", strategy=" << tabuStrategy << endl;

    initialize(scqbf);

    auto startTime = chrono::high_resolution_clock::now();

    for (totalIterations = 0; totalIterations < maxIterations; totalIterations++) {
        auto currentTime = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::seconds>(currentTime - startTime).count();
        if (elapsed >= timeLimit) {
            cout << "Time limit reached!" << endl;
            break;
        }

        bool improved = iterate(scqbf);

        if (improved) {
            iterationsWithoutImprovement = 0;
        }
        else {
            iterationsWithoutImprovement++;
        }

        applyTabuStrategy(scqbf);

        valueHistory.push_back(currentValue);

        if ((totalIterations + 1) % 100 == 0) {
            cout << "Iteration " << (totalIterations + 1) << " - Best value: "
                << bestValue << ", Current: " << currentValue << endl;
        }
    }

    cout << "Tabu Search finished. Best value found: " << bestValue << endl;
    cout << "Total iterations: " << totalIterations << endl;

    return bestSolution;
}

void TabuSearch::initialize(const SetCoverQBF& scqbf) {
    // Clear previous state
    tabuList.clear();
    tabuSet.clear();
    valueHistory.clear();

    // Initialize frequency matrix
    int n = scqbf.getNumSets();
    frequencyMatrix.assign(n, 0);

    // Generate initial solution
    currentSolution = generateInitialSolution(scqbf);
    currentValue = scqbf.evaluateSolution(currentSolution);

    // Initialize best solution
    bestSolution = currentSolution;
    bestValue = currentValue;

    cout << "Initial solution: value = " << currentValue
        << ", feasible = " << (scqbf.isFeasible(currentSolution) ? "Yes" : "No") << endl;
}

vector<int> TabuSearch::generateInitialSolution(const SetCoverQBF& scqbf) {
    return generateGreedySolution(scqbf);
}

vector<int> TabuSearch::generateGreedySolution(const SetCoverQBF& scqbf) {
    int n = scqbf.getNumSets();
    vector<int> solution(n, 0);
    set<int> uncoveredElements = scqbf.getUniverse();
    vector<bool> candidateSet(n, true);

    while (!uncoveredElements.empty()) {
        int bestCandidate = -1;
        double bestBenefit = -1e9;

        for (int i = 0; i < n; i++) {
            if (!candidateSet[i]) continue;

            // Compute benefit: covered elements + contribution to objective function
            double benefit = 0.0;

            // Count new covered elements
            const vector<int>& candidateSubset = scqbf.getSet(i);
            int newElementsCovered = 0;
            for (int element : candidateSubset) {
                if (uncoveredElements.count(element) > 0) {
                    newElementsCovered++;
                }
            }

            benefit += newElementsCovered * 100.0; // High weight for coverage
            benefit += scqbf.getLinearCoeff(i);    // Linear contribution

            // Quadratic contributions with already selected sets
            for (int j = 0; j < n; j++) {
                if (solution[j] == 1) {
                    benefit += scqbf.getQuadraticCoeff(min(i, j), max(i, j));
                }
            }

            if (benefit > bestBenefit) {
                bestBenefit = benefit;
                bestCandidate = i;
            }
        }

        if (bestCandidate == -1) break;

        solution[bestCandidate] = 1;
        candidateSet[bestCandidate] = false;

        // Update uncovered elements
        const vector<int>& selectedSet = scqbf.getSet(bestCandidate);
        for (int element : selectedSet) {
            uncoveredElements.erase(element);
        }
    }

    return solution;
}

vector<int> TabuSearch::generateRandomSolution(const SetCoverQBF& scqbf) {
    int n = scqbf.getNumSets();
    vector<int> solution(n, 0);

    // Randomly select sets until covering all elements
    set<int> uncoveredElements = scqbf.getUniverse();
    uniform_int_distribution<int> dist(0, n - 1);

    while (!uncoveredElements.empty()) {
        int randomSet = dist(rng);
        if (solution[randomSet] == 0) {
            solution[randomSet] = 1;

            const vector<int>& selectedSet = scqbf.getSet(randomSet);
            for (int element : selectedSet) {
                uncoveredElements.erase(element);
            }
        }
    }

    return solution;
}

bool TabuSearch::iterate(const SetCoverQBF& scqbf) {
    vector<Move> neighborhood = generateNeighborhood(scqbf, currentSolution);

    if (neighborhood.empty()) {
        return false; // No valid moves
    }

    Move selectedMove = selectMove(scqbf, neighborhood);

    if (selectedMove.variable == -1) {
        return false; // No valid move found
    }

    applyMove(currentSolution, selectedMove);
    currentValue += selectedMove.delta;

    addToTabuList(selectedMove);

    updateFrequencyMatrix(currentSolution);

    bool improved = false;
    if (currentValue > bestValue) {
        updateBestSolution(currentSolution, currentValue);
        improved = true;
    }

    return improved;
}

vector<TabuSearch::Move> TabuSearch::generateNeighborhood(const SetCoverQBF& scqbf,
    const vector<int>& solution) {
    vector<Move> neighborhood;
    int n = static_cast<int>(solution.size());

    // Flip operator: swap 0->1 or 1->0
    for (int i = 0; i < n; i++) {
        Move move;
        move.variable = i;
        move.from_value = solution[i];
        move.to_value = 1 - solution[i];
        move.delta = calculateMoveDelta(scqbf, solution, move);

        // Check if the move results in a feasible solution
        vector<int> testSolution = solution;
        applyMove(testSolution, move);

        // Check if the Tabu Search is currently allowing infeasible solutions (i.e. if using Strategic Oscillation)
        // If not, only accept solution if it is feasible
        if (isAllowingInfeasibleSolutions || scqbf.isFeasible(testSolution)) {
            neighborhood.push_back(move);
        }
    }

    return neighborhood;
}

TabuSearch::Move TabuSearch::selectMove(const SetCoverQBF& scqbf, const vector<Move>& moves) {
    switch (searchMethod) {
    case BEST_IMPROVING:
        return selectBestImproving(scqbf, moves);
    default:
        return selectFirstImproving(scqbf, moves);
    }
}

TabuSearch::Move TabuSearch::selectFirstImproving([[maybe_unused]] const SetCoverQBF& scqbf,
    const vector<Move>& moves) {
    for (const auto& move : moves) {
        if (!isTabu(move) || aspirationCriterion(move, currentValue + move.delta)) {
            return move;
        }
    }

    // If all moves are tabu, select the least tabu
    if (!moves.empty()) {
        return moves[0];
    }

    return Move();
}

TabuSearch::Move TabuSearch::selectBestImproving([[maybe_unused]] const SetCoverQBF& scqbf,
    const vector<Move>& moves) {
    Move bestMove;
    double bestDelta = -1e9;
    bool foundNonTabu = false;

    for (const auto& move : moves) {
        if (!isTabu(move) || aspirationCriterion(move, currentValue + move.delta)) {
            if (move.delta > bestDelta) {
                bestDelta = move.delta;
                bestMove = move;
                foundNonTabu = true;
            }
        }
    }

    // If no non-tabu move found, pick the best tabu
    if (!foundNonTabu && !moves.empty()) {
        bestMove = *max_element(moves.begin(), moves.end(),
            [](const Move& a, const Move& b) { return a.delta < b.delta; });
    }

    return bestMove;
}

bool TabuSearch::isTabu(const Move& move) const {
    return tabuSet.find(move) != tabuSet.end();
}

void TabuSearch::addToTabuList(const Move& move) {
    // Reverse move is added to tabu list
    Move reverseMove(move.variable, move.to_value, move.from_value, -move.delta);

    tabuList.push_back(reverseMove);
    tabuSet.insert(reverseMove);

    // Keep tabu list size
    while (static_cast<int>(tabuList.size()) > tabuTenure) {
        Move oldMove = tabuList.front();
        tabuList.pop_front();
        tabuSet.erase(oldMove);
    }
}

bool TabuSearch::aspirationCriterion([[maybe_unused]] const Move& move, double newValue) const {
    // Aspiration criterion: accept tabu move if it leads to the best solution found
    return newValue > bestValue;
}

void TabuSearch::applyTabuStrategy([[maybe_unused]] const SetCoverQBF& scqbf) {

    // This method will be used when multiple strategies are implemented.
    // The default is to do nothing for the STANDARD strategy.
    // For other strategies, implement the corresponding methods and call them here.

    switch (tabuStrategy) {
    // case PROBABILISTIC:
    //     applyProbabilisticTS(scqbf);
    //     break;
    // case INTENSIFICATION_RESTART:
    //     applyIntensificationRestart(scqbf);
    //     break;
    // case INTENSIFICATION_NEIGHBORHOOD:
    //     applyIntensificationNeighborhood(scqbf);
    //     break;
    // case DIVERSIFICATION_RESTART:
    //     applyDiversificationRestart(scqbf);
    //     break;
    case STRATEGIC_OSCILLATION:
        applyStrategicOscillation(scqbf);
        break;
    default:
        // Default: Standard Tabu Search
        break;
    }
}


void TabuSearch::applyStrategicOscillation(const SetCoverQBF& scqbf) {
    // If the algorythm is iterating without any improvement for a while, create oscillation to allow infeasible solutions
    // This may allow for better exploration of the search space
    if (iterationsWithoutImprovement > 60 && !isInOscillationPhase) {
        isAllowingInfeasibleSolutions = true;
        isInOscillationPhase = true;
        oscillationPhaseCounter = 0;
    }

    if (isInOscillationPhase) {
        oscillationPhaseCounter += 1;

        // If is already oscillating for a few iterations, end oscillation phase and return to secure and feasible solutions
        if (oscillationPhaseCounter > 30) {
            isAllowingInfeasibleSolutions = false;
            oscillationPhaseCounter = 0;
            currentSolution = repairSolution(scqbf, currentSolution);
            currentValue = scqbf.evaluateSolution(currentSolution);
        }
    }
}

double TabuSearch::calculateMoveDelta(const SetCoverQBF& scqbf, const vector<int>& solution, const Move& move) const {
    // Compute change in objective function
    double delta = 0.0;
    int i = move.variable;

    if (move.to_value == 1) {
        // Adding set i
        delta += scqbf.getLinearCoeff(i);

        for (size_t j = 0; j < solution.size(); j++) {
            if (static_cast<int>(j) != i && solution[j] == 1) {
                delta += scqbf.getQuadraticCoeff(min(i, static_cast<int>(j)), max(i, static_cast<int>(j)));
            }
        }
    }
    else {
        // Removing set i
        delta -= scqbf.getLinearCoeff(i);

        for (size_t j = 0; j < solution.size(); j++) {
            if (static_cast<int>(j) != i && solution[j] == 1) {
                delta -= scqbf.getQuadraticCoeff(min(i, static_cast<int>(j)), max(i, static_cast<int>(j)));
            }
        }
    }

    return delta;
}

void TabuSearch::applyMove(vector<int>& solution, const Move& move) {
    solution[move.variable] = move.to_value;
}

void TabuSearch::updateBestSolution(const vector<int>& solution, double value) {
    bestSolution = solution;
    bestValue = value;
    cout << "New best solution found: " << value << endl;
}

void TabuSearch::updateFrequencyMatrix(const vector<int>& solution) {
    for (size_t i = 0; i < solution.size(); i++) {
        if (solution[i] == 1) {
            frequencyMatrix[i]++;
        }
    }
}

vector<int> TabuSearch::repairSolution(const SetCoverQBF& scqbf, const vector<int>& solution) {
    if (scqbf.isFeasible(solution)) {
        return solution;
    }

    vector<int> repairedSolution = solution;
    set<int> uncoveredElements = scqbf.getUniverse();

    // Remove covered elements
    for (size_t i = 0; i < repairedSolution.size(); i++) {
        if (repairedSolution[i] == 1) {
            const vector<int>& subset = scqbf.getSet(static_cast<int>(i));
            for (int element : subset) {
                uncoveredElements.erase(element);
            }
        }
    }

    // Add sets to cover missing elements
    while (!uncoveredElements.empty()) {
        int bestSet = -1;
        int maxCoverage = 0;

        for (size_t i = 0; i < repairedSolution.size(); i++) {
            if (repairedSolution[i] == 0) {
                const vector<int>& subset = scqbf.getSet(static_cast<int>(i));
                int coverage = 0;
                for (int element : subset) {
                    if (uncoveredElements.count(element) > 0) {
                        coverage++;
                    }
                }

                if (coverage > maxCoverage) {
                    maxCoverage = coverage;
                    bestSet = static_cast<int>(i);
                }
            }
        }

        if (bestSet == -1) break;

        repairedSolution[bestSet] = 1;
        const vector<int>& selectedSet = scqbf.getSet(bestSet);
        for (int element : selectedSet) {
            uncoveredElements.erase(element);
        }
    }

    return repairedSolution;
}

bool TabuSearch::needsRepair(const SetCoverQBF& scqbf, const vector<int>& solution) const {
    return !scqbf.isFeasible(solution);
}
