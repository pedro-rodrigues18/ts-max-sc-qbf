#ifndef SC_QBF_HPP
#define SC_QBF_HPP

#include <string>
#include <vector>
#include <set>

using namespace std;

class SetCoverQBF {
public:
    string path;
    int m = 0; // Number of variables / sets
    vector<int> variables;
    vector<vector<int>> A;// Triangular matrix of coefficients
    vector<vector<int>> sets; // Sets of elements

    SetCoverQBF(string path);

    double evaluateSolution(const vector<int>& solution) const;
    bool isFeasible(const vector<int>& solution) const;
    set<int> getUniverse() const;
    void printProblem() const;

    int getNumSets() const;
    const vector<int>& getSet(int index) const;

    double getLinearCoeff(int i) const;

    double getQuadraticCoeff(int i, int j) const;

private:
    void readFile();
};

#endif