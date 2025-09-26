#include <iostream>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <thread>
#include <mutex>
#include <vector>
#include <future>
#include <iomanip>
#include <sstream>
#include <map>
#include "sc-qbf/sc_qbf.hpp"
#include "tabu-search/ts.hpp"

std::mutex results_mutex;

struct TabuExperimentResult {
    std::string instance;
    std::string config;
    double value;
    int time_seconds;
    bool feasible;
    int iterations_completed;
    double convergence_iteration;
};

std::vector<TabuExperimentResult> all_results;

void writeResults(const std::string& filename) {
    std::ofstream file(filename);
    file << "Instance,Configuration,Value,Time_Seconds,Feasible,Iterations,Convergence_Iteration\n";
    for (const auto& r : all_results) {
        file << r.instance << "," << r.config << ","
            << std::fixed << std::setprecision(2) << r.value << ","
            << r.time_seconds << ","
            << (r.feasible ? "Yes" : "No") << ","
            << r.iterations_completed << ","
            << std::fixed << std::setprecision(0) << r.convergence_iteration << "\n";
    }
}

TabuExperimentResult runSingleConfig(const std::string& instPath, const std::string& instName,
    const std::string& cfgName, TabuSearch::SearchMethod sm,
    TabuSearch::TabuStrategy ts, int tenure) {
    TabuExperimentResult r{ instName, cfgName, -1, -1, false, 0, 0 };

    try {
        SetCoverQBF scqbf(instPath);
        TabuSearch tabuSearch(tenure, 10000, 1800, sm, ts);

        auto start = std::chrono::high_resolution_clock::now();
        auto sol = tabuSearch.run(scqbf);
        auto end = std::chrono::high_resolution_clock::now();

        r.value = scqbf.evaluateSolution(sol);
        r.feasible = scqbf.isFeasible(sol);
        r.time_seconds = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
        r.iterations_completed = tabuSearch.getTotalIterations();

        // Find convergence iteration
        auto history = tabuSearch.getValueHistory();
        double best_value = tabuSearch.getBestValue();
        for (size_t i = 0; i < history.size(); i++) {
            if (std::abs(history[i] - best_value) < 1e-6) {
                r.convergence_iteration = static_cast<double>(i + 1);
                break;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error in " << instName << ": " << e.what() << std::endl;
    }
    return r;
}

void logResults(const std::string& instName, const std::string& baseName, const std::vector<TabuExperimentResult>& local_results) {
    std::ofstream log("logs/" + baseName + ".log");

    log << "Running Tabu Search on instance: " << instName << "\n";
    log << "Configurations tested: " << local_results.size() << "\n\n";

    const std::string indent = "    ";

    log << indent
        << std::left 
        << std::setw(27) << "Configuration"
        << std::setw(12) << "Value"
        << std::setw(10) << "Time(s)"
        << std::setw(10) << "Feasible"
        << std::setw(8) << "Iter"
        << std::setw(8) << "Conv Iter" << std::endl;

    log << indent << std::string(76, '-') << std::endl;

    for (const auto& r : local_results) {
        log << indent
            << std::left 
            << std::setw(27) << r.config
            << std::setw(12) << std::fixed << std::setprecision(2) << r.value
            << std::setw(10) << std::setprecision(1) << r.time_seconds
            << std::setw(10) << (r.feasible ? "Yes" : "No")
            << std::setw(8) << r.iterations_completed
            << std::setw(8) << r.convergence_iteration << std::endl;
    }

    log.close();
}


void runInstance(const std::string& instPath, const std::string& instName) {
    int T1 = 7;   // Small tabu tenure
    int T2 = 15;  // Large tabu tenure

    std::vector<std::tuple<std::string, TabuSearch::SearchMethod, TabuSearch::TabuStrategy, int>> configs = {
        {"STANDARD", TabuSearch::FIRST_IMPROVING, TabuSearch::STANDARD, T1},
        {"STANDARD+BEST", TabuSearch::BEST_IMPROVING, TabuSearch::STANDARD, T1},
        {"STANDARD+TENURE", TabuSearch::FIRST_IMPROVING, TabuSearch::STANDARD, T2},
        {"STRATEGIC_OSCILLATION", TabuSearch::FIRST_IMPROVING, TabuSearch::STRATEGIC_OSCILLATION, T1},
        {"INTENSIFICATION_RESTART", TabuSearch::FIRST_IMPROVING, TabuSearch::INTENSIFICATION_RESTART, T1}
    };

    std::string baseName = instName.substr(0, instName.find_last_of("."));

    std::vector<std::future<TabuExperimentResult>> futures;
    for (auto& [cfgName, sm, ts, tenure] : configs) {
        futures.push_back(std::async(std::launch::async, [&, cfgName, sm, ts, tenure]() {
            return runSingleConfig(instPath, instName, cfgName, sm, ts, tenure);
            }));
    }

    std::vector<TabuExperimentResult> local_results;
    for (auto& f : futures) local_results.push_back(f.get());

    // Save log file
    logResults(instName, baseName, local_results);

    // Show results on screen
    std::cout << "\n=== RESULTS FOR " << instName << " ===" << std::endl;
    std::cout << std::left << std::setw(27) << "Configuration"
        << std::setw(12) << "Value"
        << std::setw(10) << "Time(s)"
        << std::setw(10) << "Feasible"
        << std::setw(8) << "Iter"
        << std::setw(8) << "Conv Iter" << std::endl;
    std::cout << std::string(76, '-') << std::endl;

    for (auto& r : local_results) {
        std::cout << std::left << std::setw(27) << r.config
            << std::setw(12) << std::fixed << std::setprecision(2) << r.value
            << std::setw(10) << r.time_seconds
            << std::setw(10) << (r.feasible ? "Yes" : "No")
            << std::setw(8) << r.iterations_completed
            << std::setw(8) << static_cast<int>(r.convergence_iteration) << std::endl;
    }

    {
        std::lock_guard<std::mutex> lock(results_mutex);
        all_results.insert(all_results.end(), local_results.begin(), local_results.end());
    }
}

void runAllInstances(const std::vector<std::string>& instances) {
    unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
    // unsigned int num_threads = 1;
    std::cout << "Using " << num_threads << " thread(s).\n";

    std::atomic<size_t> next(0);
    std::vector<std::future<void>> futures;

    for (unsigned int t = 0; t < num_threads; t++) {
        futures.push_back(std::async(std::launch::async, [&]() {
            while (true) {
                size_t idx = next.fetch_add(1);
                if (idx >= instances.size()) break;

                std::cout << "\nProcessing instance " << (idx + 1) << "/"
                    << instances.size() << ": " << instances[idx] << std::endl;

                runInstance("instances/" + instances[idx], instances[idx]);
            }
            }));
    }
    for (auto& f : futures) f.wait();
}

std::vector<std::string> setupInstances(const std::string& path) {
    std::vector<std::string> insts;
    for (auto& e : std::filesystem::directory_iterator(path))
        if (e.is_regular_file()) insts.push_back(e.path().filename().string());
    std::sort(insts.begin(), insts.end());
    return insts;
}

int main() {
    std::filesystem::create_directory("logs");
    std::string path = "instances/";

    auto instances = setupInstances(path);
    if (instances.empty()) {
        std::cerr << "No instances found in " << path << std::endl;
        return 1;
    }

    std::cout << "Instances found: " << instances.size() << std::endl;

    // Test mode: run only one instance
    bool test = false;

    if (test && !instances.empty()) {
        std::string instance = instances[0];
        std::cout << "TEST MODE: Running only one instance: " << instance << std::endl;
        std::cout << "Configurations: 3 (STANDARD, STANDARD+BEST, STANDARD+TENURE)" << std::endl;
        std::cout << "Time limit per configuration: 30 minutes" << std::endl;
        std::cout << "Starting test...\n" << std::endl;

        runAllInstances({ instance });

        std::string timestamp = std::to_string(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
        std::string results_filename = "logs/tabu_test_results_" + timestamp + ".csv";
        writeResults(results_filename);

        std::cout << "\n=== TEST FINISHED ===" << std::endl;
        std::cout << "Results saved to: " << results_filename << std::endl;
        std::cout << "Detailed log: logs/" << instance.substr(0, instance.find_last_of(".")) << ".log" << std::endl;

        if (!all_results.empty()) {
            auto best_result = *std::max_element(all_results.begin(), all_results.end(),
                [](const TabuExperimentResult& a, const TabuExperimentResult& b) {
                    return a.value < b.value;
                });

            std::cout << "\nBEST RESULT:" << std::endl;
            std::cout << "Configuration: " << best_result.config << std::endl;
            std::cout << "Value: " << std::fixed << std::setprecision(2) << best_result.value << std::endl;
            std::cout << "Time: " << best_result.time_seconds << " seconds" << std::endl;
            std::cout << "Convergence: iteration " << static_cast<int>(best_result.convergence_iteration) << std::endl;
        }

    }
    else {
        std::cout << "FULL MODE: Running all " << instances.size() << " instances..." << std::endl;
        runAllInstances(instances);

        std::string timestamp = std::to_string(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
        std::string results_filename = "logs/tabu_full_results_" + timestamp + ".csv";
        writeResults(results_filename);
        std::cout << "All results saved to: " << results_filename << std::endl;
    }

    return 0;
}
