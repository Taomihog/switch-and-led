// Coauthor: Gemini, Github Copilot, Cursor

#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <climits>

#include "SwitchBlackbox.hpp"
#include "ThreadSafeQueue.hpp"

//internally linked opt_t obj.
struct opt_t {
    int target_count = 0;
    uint8_t N = 0; // Switches
    uint8_t M = 0; // Outputs
    uint8_t V = 0; // Vertices
    const std::string getfilename() const {
        return "N" + std::to_string(N) + "_M" + std::to_string(M) + "_V" + std::to_string(V);
    }
};

struct SharedState {
    std::atomic<int> cnt{0};
    std::mutex mtx;
    // more field here
};

void worker(const opt_t& opt, SharedState& shared, int id, BlockingQueue<SwitchBlackbox::Statistics>& queue) {
    SwitchBlackbox sbb(opt.N, opt.M, opt.V);
    const int full_unique = 1 << opt.N;

    while (shared.cnt.fetch_add(1, std::memory_order_relaxed) < opt.target_count) {
        SwitchBlackbox::Statistics s;
        do {
            sbb.generate_random();
        } while(!sbb.filter());
        std::lock_guard<std::mutex> lock(shared.mtx);
        auto& fc = sbb.fc;
        std::cout << std::endl << "Core id:" << id << std::endl;
        std::cout << "fc: " << fc[0] << " " << fc[1] << " " << fc[2] << " " << fc[3] << " " << fc[4] << " " << std::endl;
        for (auto& v: fc) {v = 0;}
        s = sbb.stats(true); // Do something to s?
        sbb.print_human_readable();
        queue.push(s);
    }
}

void writer(BlockingQueue<SwitchBlackbox::Statistics>& queue, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::out | std::ios::app);
    if (!outFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    while (true) {
        std::optional<SwitchBlackbox::Statistics> item = queue.pop();
        if (!item.has_value()) {
            break; 
        }
        outFile << item.value().human_readable << std::endl;
    }
    outFile.close();
    std::cout << "Writing complete. Thread joined safely." << std::endl;
}

int compute(opt_t opt, std::string filename) {
    BlockingQueue<SwitchBlackbox::Statistics> logQueue;
    std::thread writerThread(writer, std::ref(logQueue), std::ref(filename));

    SharedState shared;
    std::vector<std::thread> threads;
    unsigned int cores = std::thread::hardware_concurrency();
    std::cout << "#cores: " << cores << std::endl;
    int num_workers = cores - 2;
    threads.reserve(num_workers);

    for (int i = 0; i < num_workers; ++i) {
        threads.emplace_back(worker, std::cref(opt), std::ref(shared), i, std::ref(logQueue));
    }
    for (auto& t : threads) {
        t.join();
    }
    logQueue.shutdown();  // Signal the write thread that no more data is coming, optional has no value
    writerThread.join();  // Wait for the write thread to finish writing remaining items

    return 0;
}

int read_then_analyze(opt_t opt, std::string filename, std::string outputFilename) {
    // 1. read data
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return 1;
    }
    std::vector<std::string> all_lines;
    std::string line;
    while (std::getline(inputFile, line)) {
        all_lines.push_back(line);
    }
    inputFile.close();

    // 2. calculate stats
    std::vector<SwitchBlackbox::Statistics> all_stats{};
    SwitchBlackbox sbb(opt.N, opt.M, opt.V);

    for(auto l : all_lines) {
        for (auto& v: sbb.fc) {v = 0;}
        sbb.generate_by_string(l);
        if (!sbb.filter()) {
            std::cerr << "Error: the read is not a valid circuit.";
            auto fc = sbb.fc;
            std::cout << "fc: " << fc[0] << " " << fc[1] << " " << fc[2] << " " << fc[3] << " " << fc[4] << " " << std::endl;
            return 1;
        }
        SwitchBlackbox::Statistics s = sbb.stats(false);
        all_stats.push_back(s);
    }

    // 3. write result
    std::ofstream outFile(outputFilename, std::ios::out);
    if (!outFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return 1;
    }

    outFile << "pool_hash,tt_hash,mask,sum_relevance,sum_distance1,sum_distance2,stdev" << std::endl;
    for(auto stats : all_stats) {
        for (auto s: stats.ttp) {
            outFile << stats.hash1 << ',' 
                    << stats.hash2 << ',' 
                    << utils::mask2string(s.mask, opt.V) << ',' 
                    << s.sumrel << ',' 
                    << s.sum1 << ',' 
                    << s.sum2 << ',' 
                    << s.sd << std::endl;
        }
        }
    outFile.close();
    std::cout << "result written to " << outputFilename << std::endl;

    return 0;
}

int debug(opt_t opt) {
    std::cout << "single core debug mode, might be slow" << std::endl;
    SwitchBlackbox sbb(opt.N, opt.M, opt.V);
    auto& fc = sbb.fc;
    for (auto &v : fc) {v = 0;}

    do {
        sbb.generate_random();
    } while(!sbb.filter()); // debug if filter catches inconsistency
    
    std::cout << "fc: " << fc[0] << " " << fc[1] << " " << fc[2] << " " << fc[3] << " " << fc[4] << " " << std::endl;
    SwitchBlackbox::Statistics s = sbb.stats(true);  // debug if stats catches inconsistency
    std::cout << "hash1: " << s.hash1 << ", hash2: " << s.hash2 << std::endl;
    std::cout << s.human_readable << std::endl;
    return 0;
}

static uint64_t binaryStringToUint64(const std::string& binStr) {
    uint64_t result = 0;
    
    // Determine where the binary digits start
    size_t startIdx = (binStr.substr(0, 2) == "0b" || binStr.substr(0, 2) == "0B") ? 2 : 0;
    
    for (size_t i = startIdx; i < binStr.length(); ++i) {
        result <<= 1; // Shift existing bits left
        if (binStr[i] == '1') {
            result |= 1; // Set the last bit to 1
        } else if (binStr[i] != '0') {
            throw std::invalid_argument("Invalid character in binary string");
        }
    }
    return result;
}

int verify(opt_t opt, std::string filename, int target_line, uint64_t mask) {
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return 1;
    }
    
    int currentLine = 1;
    // Sequential scan to find line i
    std::string config_str;
    while (std::getline(inputFile, config_str)) {
        if (currentLine == target_line) {
            break; 
        }
        currentLine++;
    }

    std::stringstream ss(config_str);
    char discard; // To skip '(', ')', and ','
    int v1, v2, v3;
    if (ss >> discard >> v1 >> discard >> v2 >> discard >> v3 >> discard) {
        assert (opt.N == static_cast<uint8_t>(v1) || opt.M == static_cast<uint8_t>(v2) || opt.V == static_cast<uint8_t>(v3));
    }

    // TODO: should change to read line X of file N5_M8_V13.txt
    // const std::string config_str = "(5, 8, 14)"
    // " 3B- 4Bo 3B+ 0A- 0B+ 1B- 4A- 3Bo 2B+ 4Ao 2B- 2A- 1A- 4B- 3A- 0Ao 0A+ 1Ao 0B- 3Ao 2A+ 4B+ 4A+ 3A+ 0Bo 1B+ 1Bo 2Ao 1A+ Inp 2Bo"
    // " | Walls: 2 6 7 9 10 12 13 16 19 20 21 25 29"
    // " | Hash1: 14341035504243165437"
    // " | Hash2: 9942663121443662040";
    // const uint64_t mask = 0b1111000011011;
    
    SwitchBlackbox sbb(opt.N, opt.M, opt.V);
    sbb.print_pretty_final_format(config_str, mask);
    return 0;
}

enum class Mode
{
    Read,
    Compute,
    Debug,
    Verify
};

Mode parse_mode(const std::string& opt)
{
    if (opt == "-r") return Mode::Read;
    if (opt == "-c") return Mode::Compute;
    if (opt == "-d") return Mode::Debug;
    if (opt == "-v") return Mode::Verify;

    throw std::runtime_error("Invalid option");
}

int main(int argc, char** argv) {
    auto print_help = []() {
        std::cerr << "usage: main.exe "
            << "-r 5 8 17 (read results then analyze) | "
            << "-c 5 8 14 (compute then save results) | "
            << "-d 5 8 13 (debug) | "
            << "-v 5 8 14 1543 0b1111000011011 (verify a result)\n";
    };

    try {
        Mode mode = parse_mode(argv[1]);

        opt_t opt {100000, 0, 0, 0};
        int line = 0;
        if (mode != Mode::Verify) {
            assert(argc == 5);
        } else {
            assert(argc == 7);
        }
        opt.N = std::stoi(argv[2]);
        opt.M = std::stoi(argv[3]);
        opt.V = std::stoi(argv[4]);
        const std::string filename = opt.getfilename() + ".txt";
        const std::string outputFilename = "stats_" + opt.getfilename() + ".csv";

        switch (mode) {
        case Mode::Read:
            std::cout << "read mode\n";
            return read_then_analyze(opt, filename, outputFilename);

        case Mode::Compute:
            std::cout << "compute mode\n";
            return compute(opt, filename); 

        case Mode::Debug:
            std::cout << "debug mode\n";
            return debug(opt);

        case Mode::Verify:
            std::cout << "verify mode\n";
            return verify(opt, filename, std::stoi(argv[5]), binaryStringToUint64(argv[6]));
        }
    } catch (const std::exception&) {
        print_help();
        return 1;
    }
}
