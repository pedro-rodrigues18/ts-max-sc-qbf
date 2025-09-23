#include "sc_qbf.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

SetCoverQBF::SetCoverQBF(string path) : path(path) {
    readFile();
}

void SetCoverQBF::readFile() {
    ifstream file(path);
    string line;

    if (!file.is_open()) {
        cerr << "Error opening file: " << path << endl;
        return;
    }

    if (getline(file, line)) {
        stringstream ss(line);
        ss >> this->m;
    }

    this->variables.assign(this->m, 0);
    this->sets.assign(this->m, {});
    this->A.assign(this->m, {});

    vector<int> sizes(this->m);
    if (getline(file, line)) {
        stringstream ss(line);
        for (int i = 0; i < this->m; i++) {
            ss >> sizes[i];
        }
    }

    // 3. Read sets S1, S2, ..., Sm
    for (int i = 0; i < this->m; i++) {
        if (getline(file, line)) {
            stringstream ss(line);
            int elem;
            while (ss >> elem) {
                this->sets[i].push_back(elem);
            }
            // Verify if the read size matches the expected size
            if ((int)this->sets[i].size() != sizes[i]) {
                cerr << "Error: size of S" << (i + 1)
                    << " differs of especified ("
                    << sizes[i] << ", "
                    << this->sets[i].size() << ")." << endl;
            }
        }
    }

    // 4. Read triangular matrix A
    for (int i = 0; i < this->m; i++) {
        if (getline(file, line)) {
            stringstream ss(line);
            int val;
            while (ss >> val) {
                this->A[i].push_back(val);
            }
        }
    }

    file.close();
}

double SetCoverQBF::evaluateSolution(const vector<int>& solution) const {
    double totalValue = 0.0;

    // Linear terms: first element of each row in A
    for (int i = 0; i < this->m; i++) {
        if (solution[i] == 1) {
            totalValue += this->A[i][0]; // First element of row 1 (linear term)
        }
    }

    // Quadratic terms: upper triangular part of A
    for (int i = 0; i < this->m; i++) {
        if (solution[i] == 1) {
            // For each row i, we start from index 1 (which corresponds to element A[i][i+1])
            for (int j = i + 1; j < this->m; j++) {
                if (solution[j] == 1) {
                    // The coefficient A[i][j] is stored in A[i][j-i]
                    // That is, A[i][1] corresponds to A[i][i+1], A[i][2] to A[i][i+2], etc.
                    int coeff_index = j - i;
                    if (coeff_index < static_cast<int>(this->A[i].size())) {
                        totalValue += this->A[i][coeff_index];
                    }
                }
            }
        }
    }

    return totalValue;
}

bool SetCoverQBF::isFeasible(const vector<int>& solution) const {
    set<int> universe;
    for (const auto& subset : this->sets) {
        for (int element : subset) {
            universe.insert(element);
        }
    }

    for (int element : universe) {
        bool covered = false;
        for (int i = 0; i < this->m; i++) {
            if (solution[i] == 1) {
                for (int elem : this->sets[i]) {
                    if (elem == element) {
                        covered = true;
                        break;
                    }
                }
                if (covered) break;
            }
        }

        if (!covered) {
            return false;
        }
    }

    return true;
}

set<int> SetCoverQBF::getUniverse() const {
    set<int> universe;
    for (const auto& subset : this->sets) {
        for (int element : subset) {
            universe.insert(element);
        }
    }
    return universe;
}

int SetCoverQBF::getNumSets() const { return m; }

const vector<int>& SetCoverQBF::getSet(int index) const { return sets[index]; }

double SetCoverQBF::getLinearCoeff(int i) const {
    return ((i >= 0) && (i < static_cast<int>(A.size())) && (A[i].size() > 0)) ? A[i][0] : 0.0;
}

double SetCoverQBF::getQuadraticCoeff(int i, int j) const {
    if (i > j) swap(i, j);
    if ((i >= 0) && (i < static_cast<int>(A.size()))) {
        int coeff_index = j - i;
        if ((coeff_index > 0) && (coeff_index < static_cast<int>(A[i].size()))) {
            return A[i][coeff_index];
        }
    }
    return 0.0;
}

void SetCoverQBF::printProblem() const {
    cout << "Number of sets (m): " << this->m << endl;
    cout << "Sets:" << endl;
    for (const auto& s : this->sets) {
        for (int elem : s) {
            cout << elem << " ";
        }
        cout << endl;
    }
    cout << "Matrix A:" << endl;
    for (const auto& row : this->A) {
        for (int val : row) {
            cout << val << " ";
        }
        cout << endl;
    }
}
